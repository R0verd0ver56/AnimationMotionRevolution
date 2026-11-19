#pragma once

// Vertical root motion on 1.6.1170.
//
// The game projects animation motion onto the ground plane while a character is in the on-ground
// state, so the Z part of an `animmotion` curve (launchers such as Vergil's Rising Star / Rising Sun
// or Dante's power attack, ~220-250 units up) never lifts the actor. The upstream "report swimming"
// trick did not lift anyone on 1.6.1170 in testing, so this drives the height directly:
//
//   * the translation hook turns the sampled cumulative Z of the current clip into a vertical speed
//     (game units per second) for that actor's character controller;
//   * the on-ground physics step (already replaced by AMR) switches the controller to in-air when the
//     curve rises, and sets its vertical velocity from the curve;
//   * the in-air physics step keeps following the curve (up and down) while the curve is active.
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
	inline constexpr auto kSampleLifetime = std::chrono::milliseconds(150);
	// Game units per Havok unit.
	inline constexpr float kHavokToGame = 69.991249F;
	// Below this rise speed (game units / s) the actor is not pulled off the ground.
	inline constexpr float kMinLiftSpeed = 5.0F;

	struct Track
	{
		Clock::time_point stamp{};
		const void*       clip{ nullptr };
		float             prevTime{ 0.0F };
		float             prevZ{ 0.0F };
		float             speed{ 0.0F };  // game units / s, + up
		bool              hasPrev{ false };
		bool              active{ false };  // current clip moves vertically
		Clock::time_point lastLog{};
	};

	struct Registry
	{
		std::mutex                                               lock;
		std::unordered_map<const RE::hkpCharacterContext*, Track> tracks;
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
		const auto* context = ContextOf(a_character);
		if (!context) {
			return;
		}
		auto& r = Get();
		std::lock_guard guard(r.lock);
		if (!a_clipHasVerticalMotion) {
			r.tracks.erase(context);
			return;
		}
		auto&       t = r.tracks[context];
		const float dt = a_motionTime - t.prevTime;
		if (t.clip != a_clip || !t.hasPrev || dt < -0.25F) {
			// new clip (or clip restarted): no speed until the next sample
			t.clip = a_clip;
			t.speed = 0.0F;
		} else if (dt < 0.0005F) {
			// The game samples a clip more than once per frame (previous and current time), so a
			// sample at the same or a slightly earlier time just keeps the current speed.
			t.stamp = Clock::now();
			return;
		} else {
			t.speed = (a_translation.z - t.prevZ) / dt;
		}
		t.prevTime = a_motionTime;
		t.prevZ = a_translation.z;
		t.hasPrev = true;
		t.active = true;
		t.stamp = Clock::now();
	}

	// Translation hook, vanilla / no motion branch.
	inline void Clear(RE::Character* a_character)
	{
		const auto* context = ContextOf(a_character);
		if (!context) {
			return;
		}
		auto& r = Get();
		std::lock_guard guard(r.lock);
		r.tracks.erase(context);
	}

	// Physics hooks: vertical speed wanted by the curve right now, if any.
	inline bool Speed(const RE::hkpCharacterContext* a_context, float& a_speed, bool& a_logNow)
	{
		auto& r = Get();
		std::lock_guard guard(r.lock);
		const auto it = r.tracks.find(a_context);
		if (it == r.tracks.end()) {
			return false;
		}
		auto&      t = it->second;
		const auto now = Clock::now();
		if (!t.active || now - t.stamp > kSampleLifetime) {
			r.tracks.erase(it);
			return false;
		}
		a_speed = t.speed;
		a_logNow = now - t.lastLog > std::chrono::milliseconds(100);
		if (a_logNow) {
			t.lastLog = now;
		}
		return true;
	}

	inline void ClearAll()
	{
		auto& r = Get();
		std::lock_guard guard(r.lock);
		r.tracks.clear();
	}
}
