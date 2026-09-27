// BoundedQueue.hpp -- cola productor-consumidor acotada para el pipeline por
// tareas (--pipeline).
//
// Es el mecanismo de "colas (1:1)" del Cap. 2 (tareas heterogeneas): una etapa
// produce, la siguiente consume, y un mutex con dos variables de condicion
// protege la cola. Con capacidad 2 funciona como un ping-pong buffer: mientras
// la etapa siguiente procesa un scan, la anterior ya llena el otro, y si se
// adelanta demasiado se bloquea en vez de acumular scans en memoria.
//
// Cierre: el productor llama Close() al terminar. Pop() devuelve false cuando
// la cola esta cerrada y vacia, que es la senal de fin para el consumidor.

#pragma once

#include <condition_variable>
#include <cstddef>
#include <deque>
#include <mutex>
#include <utility>

namespace recon {

template <class T>
class BoundedQueue {
public:
    explicit BoundedQueue(std::size_t capacity) : capacity_(capacity ? capacity : 1) {}

    // Bloquea mientras la cola esta llena.
    void Push(T item) {
        std::unique_lock<std::mutex> lock(mutex_);
        not_full_.wait(lock, [&] { return items_.size() < capacity_; });
        items_.push_back(std::move(item));
        lock.unlock();
        not_empty_.notify_one();
    }

    // Bloquea mientras la cola esta vacia. false = cerrada y sin elementos.
    bool Pop(T& out) {
        std::unique_lock<std::mutex> lock(mutex_);
        not_empty_.wait(lock, [&] { return !items_.empty() || closed_; });
        if (items_.empty()) return false;
        out = std::move(items_.front());
        items_.pop_front();
        lock.unlock();
        not_full_.notify_one();
        return true;
    }

    void Close() {
        {
            std::lock_guard<std::mutex> lock(mutex_);
            closed_ = true;
        }
        not_empty_.notify_all();
    }

private:
    std::size_t capacity_;
    std::deque<T> items_;
    bool closed_ = false;
    std::mutex mutex_;
    std::condition_variable not_full_;
    std::condition_variable not_empty_;
};

}  // namespace recon
