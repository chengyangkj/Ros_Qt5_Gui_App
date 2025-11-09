#pragma once

#include <thread>
#include <atomic>

namespace rosbridge2cpp {

	class spinlock
	{
	private:
		std::atomic_flag lock_;

	public:
		spinlock()
		{
			lock_.clear(std::memory_order_release);
		}

		bool try_lock()
		{
			return !lock_.test_and_set(std::memory_order_acquire);
		}

		void lock(const bool sleep = false)
		{
			while (!try_lock())
			{
				if (sleep)
				{
					std::this_thread::yield();
				}
			}
		}

		void unlock()
		{
			lock_.clear(std::memory_order_release);
		}

	public:

		template<bool WaitForLongTask>
		class scoped_lock
		{
		private:
			spinlock& spinlock_;
			scoped_lock(scoped_lock const &);
			scoped_lock & operator=(scoped_lock const &);

		public:
			explicit scoped_lock(spinlock& sp) : spinlock_(sp)
			{
				sp.lock(WaitForLongTask);
			}

			~scoped_lock()
			{
				spinlock_.unlock();
			}
		};

		// assume the other task may take longer, so use sleep/yield
		typedef scoped_lock<true> scoped_lock_wait_for_long_task;
		// assume the other task finishes soon, so spin wait
		typedef scoped_lock<false> scoped_lock_wait_for_short_task;
	};

} // namespace rosbridge2cpp
