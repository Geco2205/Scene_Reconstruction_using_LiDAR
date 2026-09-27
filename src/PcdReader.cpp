#include "recon/PcdReader.hpp"

#include <cmath>
#include <cstdint>
#include <cstring>
#include <fstream>
#include <sstream>
#include <stdexcept>

namespace recon {
namespace {

struct Field {
    std::string name;
    int size = 0;          // bytes por elemento
    char type = 'F';       // 'I', 'U' o 'F'
    int count = 1;         // elementos por punto
    std::size_t byte_off = 0;   // offset en bytes dentro del punto (binary)
    std::size_t token_off = 0;  // indice del token en la linea (ascii)
};

std::vector<std::string> Split(const std::string& line) {
    std::vector<std::string> out;
    std::istringstream iss(line);
    std::string tok;
    while (iss >> tok) out.push_back(tok);
    return out;
}

bool Finite(float a, float b, float c) {
    return std::isfinite(a) && std::isfinite(b) && std::isfinite(c);
}

}  // namespace

PointCloudSoA ReadPcd(const std::string& path) {
    std::ifstream file(path, std::ios::binary);
    if (!file) throw std::runtime_error("No se pudo abrir el archivo: " + path);

    std::vector<std::string> names, types_raw;
    std::vector<int> sizes, counts;
    std::size_t n_points = 0;
    std::string data_format;

    // --- Header ---------------------------------------------------------
    // Se lee linea por linea hasta DATA. Despues de la linea DATA, en el caso
    // binary, el cuerpo empieza inmediatamente; por eso se usa tellg() sobre el
    // stream binario en vez de getline sobre un stream de texto.
    std::string line;
    while (std::getline(file, line)) {
        if (!line.empty() && line.back() == '\r') line.pop_back();
        if (line.empty() || line[0] == '#') continue;

        const auto tok = Split(line);
        if (tok.empty()) continue;
        const std::string& key = tok[0];

        if (key == "FIELDS") {
            names.assign(tok.begin() + 1, tok.end());
        } else if (key == "SIZE") {
            for (std::size_t i = 1; i < tok.size(); ++i) sizes.push_back(std::stoi(tok[i]));
        } else if (key == "TYPE") {
            types_raw.assign(tok.begin() + 1, tok.end());
        } else if (key == "COUNT") {
            for (std::size_t i = 1; i < tok.size(); ++i) counts.push_back(std::stoi(tok[i]));
        } else if (key == "POINTS") {
            if (tok.size() > 1) n_points = static_cast<std::size_t>(std::stoull(tok[1]));
        } else if (key == "WIDTH" && n_points == 0) {
            if (tok.size() > 1) n_points = static_cast<std::size_t>(std::stoull(tok[1]));
        } else if (key == "DATA") {
            if (tok.size() > 1) data_format = tok[1];
            break;
        }
    }

    if (names.empty()) throw std::runtime_error("PCD sin FIELDS: " + path);
    if (sizes.size() != names.size()) throw std::runtime_error("PCD: SIZE no coincide con FIELDS: " + path);
    if (counts.empty()) counts.assign(names.size(), 1);  // COUNT es opcional

    // --- Offsets --------------------------------------------------------
    std::vector<Field> fields(names.size());
    std::size_t byte_off = 0, token_off = 0;
    for (std::size_t i = 0; i < names.size(); ++i) {
        fields[i].name = names[i];
        fields[i].size = sizes[i];
        fields[i].type = (i < types_raw.size() && !types_raw[i].empty()) ? types_raw[i][0] : 'F';
        fields[i].count = (i < counts.size()) ? counts[i] : 1;
        fields[i].byte_off = byte_off;
        fields[i].token_off = token_off;
        byte_off += static_cast<std::size_t>(fields[i].size) * fields[i].count;
        token_off += static_cast<std::size_t>(fields[i].count);
    }
    const std::size_t point_step = byte_off;
    const std::size_t tokens_per_point = token_off;

    // Localizar x, y, z
    const Field* fx = nullptr;
    const Field* fy = nullptr;
    const Field* fz = nullptr;
    for (const auto& f : fields) {
        if (f.name == "x") fx = &f;
        else if (f.name == "y") fy = &f;
        else if (f.name == "z") fz = &f;
    }
    if (!fx || !fy || !fz) throw std::runtime_error("PCD sin campos x/y/z: " + path);
    if (fx->size != 4 || fy->size != 4 || fz->size != 4 || fx->type != 'F') {
        throw std::runtime_error("PCD: se esperaba x/y/z como float32: " + path);
    }

    PointCloudSoA cloud;
    cloud.reserve(n_points);

    // --- Cuerpo ---------------------------------------------------------
    if (data_format == "ascii") {
        while (std::getline(file, line)) {
            if (!line.empty() && line.back() == '\r') line.pop_back();
            if (line.empty() || line[0] == '#') continue;
            const auto tok = Split(line);
            if (tok.size() < tokens_per_point) continue;  // linea truncada
            try {
                const float px = std::stof(tok[fx->token_off]);
                const float py = std::stof(tok[fy->token_off]);
                const float pz = std::stof(tok[fz->token_off]);
                if (Finite(px, py, pz)) cloud.push_back(px, py, pz);
            } catch (const std::exception&) {
                continue;  // "nan", token invalido: se descarta el punto
            }
        }
    } else if (data_format == "binary") {
        // El cuerpo empieza justo despues del '\n' de la linea DATA.
        const std::streampos body_start = file.tellg();
        if (body_start == std::streampos(-1)) {
            throw std::runtime_error("PCD binary: no se pudo ubicar el cuerpo: " + path);
        }
        std::vector<char> buffer(point_step * n_points);
        file.read(buffer.data(), static_cast<std::streamsize>(buffer.size()));
        const std::size_t read_points =
            static_cast<std::size_t>(file.gcount()) / (point_step ? point_step : 1);

#if RECON_SOA_ALIGNED
        // Escritura por indice sin ramas: se escribe siempre y el indice solo
        // avanza si el punto es finito. Evita el chequeo de capacidad de
        // push_back y el salto mal predicho por punto.
        cloud.resize(read_points);
        float* __restrict ox = cloud.x.data();
        float* __restrict oy = cloud.y.data();
        float* __restrict oz = cloud.z.data();
        std::size_t k = 0;
        for (std::size_t i = 0; i < read_points; ++i) {
            const char* base = buffer.data() + i * point_step;
            float px, py, pz;
            std::memcpy(&px, base + fx->byte_off, 4);
            std::memcpy(&py, base + fy->byte_off, 4);
            std::memcpy(&pz, base + fz->byte_off, 4);
            ox[k] = px;
            oy[k] = py;
            oz[k] = pz;
            k += Finite(px, py, pz) ? 1 : 0;
        }
        cloud.resize(k);
#else
        for (std::size_t i = 0; i < read_points; ++i) {
            const char* base = buffer.data() + i * point_step;
            float px, py, pz;
            std::memcpy(&px, base + fx->byte_off, 4);
            std::memcpy(&py, base + fy->byte_off, 4);
            std::memcpy(&pz, base + fz->byte_off, 4);
            if (Finite(px, py, pz)) cloud.push_back(px, py, pz);
        }
#endif
    } else if (data_format == "binary_compressed") {
        throw std::runtime_error(
            "PCD binary_compressed no soportado (requiere LZF). Reconvierta el archivo a "
            "binary o ascii: " + path);
    } else {
        throw std::runtime_error("PCD: formato DATA desconocido '" + data_format + "': " + path);
    }

    return cloud;
}

}  // namespace recon
