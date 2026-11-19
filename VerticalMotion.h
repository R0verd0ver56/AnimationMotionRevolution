#pragma once

// Vertical root motion on 1.6.1170.
//
// Upstream AMR keeps the Z part of an animmotion curve by reporting the swimming state instead of
// on-ground while the sampled translation has a Z component (it wrote a flag into the shared
// on-ground state object: `charStateOnGround->unk10 = a_translation.z == 0.0`). The 1.6.1170 port
// kept the GetCharacterState hook but lost that assignment, so Z motion was always projected onto
// the ground plane and dropped.
//
// This restores it per character controller instead of through the shared state object: the
// translation hook marks a controller's context while the current motion sample has Z != 0, and the
// GetCharacterState hook reports kSwimming for marked contexts only. A mark expires on its own so an
// interrupted clip cannot leave an actor stuck in the swimming state.

#include <chrono>
#include <mutex>
#include <unordered_map>

#include "RE/Skyrim.h"

namespace amr::vertical
{
	using Clock = std::chrono::steady_clock;

	// A mark older than this no longer counts (the translation hook refreshes it every frame).
	inline constexpr auto kMarkLifetime = std::chrono::milliseconds(150);

	struct Registry
	{
		std::mutex                                                       lock;
		std::unordered_map<const RE::hkpCharacterContext*, Clock::time_point> marks;
	};

	inline Registry& Get()
	{
		static Registry registry;
		return registry;
	}

	// Called from the translation hook with the final (modified) translation of this frame.
	inline void Update(RE::Character* a_character, const RE::NiPoint3& a_translation)
	{
		if (!a_character) {
			return;
		}
		auto* controller = a_character->GetCharController();
		if (!controller) {
			return;
		}
		const RE::hkpCharacterContext* context = std::addressof(controller->context);
		auto& r = Get();
		std::lock_guard guard(r.lock);
		if (a_translation.z != 0.0F) {
			r.marks[context] = Clock::now();
		} else {
			r.marks.erase(context);
		}
	}

	// Called from the character-state hook.
	inline bool IsActive(const RE::hkpCharacterContext* a_context)
	{
		auto& r = Get();
		std::lock_guard guard(r.lock);
		const auto it = r.marks.find(a_context);
		if (it == r.marks.end()) {
			return false;
		}
		if (Clock::now() - it->second > kMarkLifetime) {
			r.marks.erase(it);
			return false;
		}
		return true;
	}

	inline void Clear()
	{
		auto& r = Get();
		std::lock_guard guard(r.lock);
		r.marks.clear();
	}
}
