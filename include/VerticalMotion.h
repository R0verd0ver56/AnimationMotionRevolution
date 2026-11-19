#pragma once

// Vertical root motion on 1.6.1170.
//
// The game projects animation motion onto the ground plane while a character is in the on-ground
// state, so the Z part of an `animmotion` curve (launchers such as Vergil's Rising Star / Rising Sun
// or Dante's power attack, ~220-250 units up) never lifts the actor. This drives the height directly:
//
//   * the translation hook turns the sampled cumulative Z of each clip into a vertical speed
//     (game units per second), tracked per character controller AND per clip;
//   * the on-ground physics step switches the controller to in-air when the curve rises, and sets
//     its vertical velocity from the curve;
//   * the in-air physics step keeps following the curve (up and down) while the curve is active.
//
// Per-clip tracking matters: Open Animation Replacer keeps a duplicate of each replaced clip
// generator ("Duplicate_MCO_ClipGenerator_...") and the game can sample both in the same frame.
// Tracking one clip per actor made them reset each other's speed, so some launches never lifted.
//
// When the curve stops (clip ends or has no more Z change) nothing is overridden any more and normal
// gravity / landing take over (CGAT's gravity annotations apply from there).

#include <chrono>
#include <mutex>
#include <unordered_map>

#include "RE/Skyrim.h"

namespace amr::vertical
{
	using Clock = std::chrono::steady_clock;

	// A sample older than this no longer counts (the translation hook refreshes it every frame).
	inline constexpr auto kSampleLifetime = std::chrono::milliseconds(100);
	// Game units per Havok unit.
	inline constexpr float kHavokToGame = 69.991249F;
	// Below this rise speed (game units / s) the actor is not pulled off the ground.
	inline constexpr float kMinLiftSpeed = 5.0F;

	struct ClipTrack
	{
		Clock::time_point stamp{};    // last sample with new motion
		float             prevTime{ 0.0F };
		float             prevZ{ 0.0F };
		float             speed{ 0.0F };  // game units / s, + up
		bool              hasSpeed{ false };
	};

	struct ActorTrack
	{
		std::unordered_map<const void*, ClipTrack> clips;
		Clock::time_point                           lastLog{};
	};

	struct Registry
	{
		std::mutex                                                     lock;
		std::unordered_map<const RE::hkpCharacterContext*, ActorTrack> actors;
	};

	inline Registry& Get()
	{
		static Registry registry;
		return registry;
	}

	inline const RE::hkpCharacterContext* ContextOf(RE::Character* a_character)
	{
		if (!a_character) {
			return nullptr;
		}
		auto* controller = a_character->GetCharController();
		return controller ? std::addressof(controller->context) : nullptr;
	}

	// Translation hook, custom (animmotion) branch: a_translation is the cumulative displacement of
	// the clip at a_motionTime.
	inline void Sample(RE::Character* a_character, const void* a_clip, float a_motionTime, const RE::NiPoint3& a_translation,
		bool a_clipHasVerticalMotion)
	{
		if (!a_clipHasVerticalMotion) {
			return;  // other clips never cancel a running curve; samples simply expire
		}
		const auto* context = ContextOf(a_character);
		if (!context) {
			return;
		}
		auto& r = Get();
		std::lock_guard guard(r.lock);
		auto&       t = r.actors[context].clips[a_clip];
		const auto  now = Clock::now();
		const float dt = a_motionTime - t.prevTime;
		if (t.stamp == Clock::time_point{} || now - t.stamp > kSampleLifetime || dt < -0.25F) {
			// first sample of this clip (or it restarted): speed comes with the next sample
			t.hasSpeed = false;
		} else if (dt < 0.0005F) {
			// The game samples a clip more than once per frame (previous and current time): keep the speed.
			t.stamp = now;
			return;
		} else {
			t.speed = (a_translation.z - t.prevZ) / dt;
			t.hasSpeed = true;
		}
		t.prevTime = a_motionTime;
		t.prevZ = a_translation.z;
		t.stamp = now;
	}

	// Physics hooks: vertical speed wanted right now (freshest clip of this actor that has a speed).
	inline bool Speed(const RE::hkpCharacterContext* a_context, float& a_speed, bool& a_logNow)
	{
		auto& r = Get();
		std::lock_guard guard(r.lock);
		const auto it = r.actors.find(a_context);
		if (it == r.actors.end()) {
			return false;
		}
		const auto       now = Clock::now();
		const ClipTrack* best = nullptr;
		for (auto c = it->second.clips.begin(); c != it->second.clips.end();) {
			if (now - c->second.stamp > kSampleLifetime) {
				c = it->second.clips.erase(c);
				continue;
			}
			if (c->second.hasSpeed && (!best || c->second.stamp > best->stamp)) {
				best = std::addressof(c->second);
			}
			++c;
		}
		if (it->second.clips.empty()) {
			r.actors.erase(it);
			return false;
		}
		if (!best) {
			return false;
		}
		a_speed = best->speed;
		a_logNow = now - it->second.lastLog > std::chrono::milliseconds(100);
		if (a_logNow) {
			it->second.lastLog = now;
		}
		return true;
	}

	// Kept for the vanilla-motion branch of the translation hook: no longer cancels anything (see Sample).
	inline void Clear(RE::Character*) {}

	inline void ClearAll()
	{
		auto& r = Get();
		std::lock_guard guard(r.lock);
		r.actors.clear();
	}
}
