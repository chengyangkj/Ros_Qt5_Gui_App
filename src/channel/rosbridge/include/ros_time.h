#pragma once
#include <chrono>

namespace rosbridge2cpp {
	/**
	 * A class that contains the same time format like the original ROSTime.
	 *
	 * sec_ contains the seconds sind 1.1.1970.
	 * nsec_ contains the rest nanoseconds from sec_ up to the given time
	 */
	class ROSTime {
	public:
		ROSTime() = default;
		ROSTime(unsigned long sec, unsigned long nsec) : sec_(sec), nsec_(nsec) {}

		static ROSTime now();

		unsigned long sec_;
		unsigned long nsec_;

		static bool use_sim_time;
		static ROSTime sim_time;

		static const std::chrono::high_resolution_clock::duration HRCEpocOffset;
	};
}
