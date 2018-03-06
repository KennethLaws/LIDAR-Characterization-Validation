#if !defined __BAH_BLOCKING_QUEUE_H_
#define __BAH_BLOCKING_QUEUE_H_

#include <queue>
#include <condition_variable>
#include <chrono>
#include <thread>
#include <type_traits>
#include <cassert>

// This class initializes to an active state, i.e. shutdown is false to start.
template < typename Item, unsigned item_limit = 64 >
class BlockingQueue : private std::queue<Item>
{
public: // Exceptions, typedefs
    struct Timeout { };
    typedef std::queue<Item> Base;

public:
    BlockingQueue() : std::queue<Item>() { }

    virtual ~BlockingQueue()
    {
        assert(this->Base::empty());
    }

    Item pop()
    {
        std::unique_lock<std::mutex> lock(_transaction_mutex);
        auto condition = [&] { return ! this->Base::empty(); };
        _item_can_pop.wait(lock, condition);
        Item ret(std::move(this->front()));
        this->Base::pop();
        _item_can_push.notify_one();
        return ret;
    }

    // Can throw Timeout
    template <typename ChronoDuration>
    Item pop(ChronoDuration timeout)
    {
        std::unique_lock<std::mutex> lock(_transaction_mutex);
        auto condition = [&] { return ! this->Base::empty(); };

        if (!_item_can_pop.wait_for(lock, timeout, condition)) {
            throw Timeout();
        }

        Item ret(std::move(this->front()));
        this->Base::pop();
        _item_can_push.notify_one();
        return ret;
    }

    void push(const Item & item)
    {
        std::unique_lock<std::mutex> lock(_transaction_mutex);

        if (this->Base::empty()) {
            this->Base::push(item);
            _item_can_pop.notify_one();
        } else {
            if (item_limit != 0) {
// These pragmas are to suppress a bug in GCC's warning mechanism.
// See https://gcc.gnu.org/bugzilla/show_bug.cgi?id=11856
// Seems this will be fixed in gcc 4.8
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wtype-limits"
                auto condition = [&] { return this->size() < item_limit; };
#pragma GCC diagnostic pop
                _item_can_push.wait(lock, condition);
            }
            this->Base::push(item);
        }
    }

    void push(Item && item)
    {
        std::unique_lock<std::mutex> lock(_transaction_mutex);

        if (this->Base::empty()) {
            this->Base::push(std::move(item));
            _item_can_pop.notify_one();
        } else {
            if (item_limit != 0) {
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wtype-limits"
                auto condition = [&] { return this->size() < item_limit; };
#pragma GCC diagnostic pop
                _item_can_push.wait(lock, condition);
            }
            this->Base::push(std::move(item));
        }
    }

    // Can throw Timeout
    template <typename ChronoDuration>
    void push(const Item & item, ChronoDuration timeout)
    {
        std::unique_lock<std::mutex> lock(_transaction_mutex);

        if (this->Base::empty()) {
            this->Base::push(item);
            _item_can_pop.notify_one();
        } else {
            if (item_limit != 0) {
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wtype-limits"
                auto condition = [&] { return this->size() < item_limit; };
#pragma GCC diagnostic pop

                if (!_item_can_push.wait_for(lock, timeout, condition)) {
                    throw Timeout();
                }
            }

            this->Base::push(item);
        }
    }

    // Can throw Timeout
    template <typename ChronoDuration>
    void push(Item && item, ChronoDuration timeout)
    {
        std::unique_lock<std::mutex> lock(_transaction_mutex);

        if (this->Base::empty()) {
            this->Base::push(std::move(item));
            _item_can_pop.notify_one();
        } else {
            if (item_limit != 0) {
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wtype-limits"
                auto condition = [&] { return this->size() < item_limit; };
#pragma GCC diagnostic pop

                if (!_item_can_push.wait_for(lock, timeout, condition)) {
                    throw Timeout();
                }
            }

            this->Base::push(std::move(item));
        }
    }

    // Use sparingly, and with caution. Races could occur.
    bool empty()
    {
        std::unique_lock<std::mutex> lock(_transaction_mutex);
        return this->Base::empty();
    }

private:
    mutable std::mutex _transaction_mutex;
    mutable std::condition_variable _item_can_push;
    mutable std::condition_variable _item_can_pop;
};


#endif
