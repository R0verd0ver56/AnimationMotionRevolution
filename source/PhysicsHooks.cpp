#include "Hooks.h"
#include "PhysicsEvents.h"
#include "VerticalMotion.h"
#include "utils/Logger.h"

#include <algorithm>
#include <mutex>
#include <unordered_map>

namespace hooks
{
    namespace
    {
        // v7 (1.7.104): the one-step ground-ray misses that happen while standing on flat ground during attack clips
        // switched the actor to in-air at once (aerial attack on the ground). Real drops still switch immediately
        // when a longer probe shows the ground is far below. The player also switches at once when the probe finds no
        // ground at all (tall ledge); otherwise the switch waits for the miss threshold below (255 player, 6 enemies).
        constexpr std::uint32_t kMissesBeforeAirPlayer = 255;  // player: only a real drop (far probe) switches to air
        constexpr std::uint32_t kMissesBeforeAirOther = 6;     // enemies: the K=6 behaviour that gave the consistent knock-up
        constexpr float         kFarProbeLength = 1.5F;      // havok units (~105 game units)
        constexpr float         kFarGroundDistance = 0.7F;   // havok units (~49 game units), beyond the 0.5 short ray

        std::mutex                                                          g_missLock;
        std::unordered_map<const RE::bhkCharacterController*, std::uint32_t> g_misses;

        void ResetMisses(const RE::bhkCharacterController* a_controller)
        {
            std::lock_guard guard(g_missLock);
            const auto it = g_misses.find(a_controller);
            if (it != g_misses.end()) {
                if (it->second >= 3) {
                    logger::info("[AMR-AIR] ray miss run of {} steps ended by a ground hit", it->second);
                }
                g_misses.erase(it);
            }
        }

        // true once the miss should really switch the actor to in-air
        bool ShouldGoInAir(const RE::bhkCharacterController* a_controller, RE::bhkWorld* a_world,
            const RE::hkVector4& a_position, const RE::CFilter& a_filter, bool a_immediateWhenNoGround)
        {
            RE::bhkPickData farPick;
            farPick.rayInput.from = a_position;
            farPick.rayInput.to = a_position - RE::hkVector4{ 0.0F, 0.0F, kFarProbeLength, 0.0F };
            farPick.rayInput.filterInfo = a_filter;
            a_world->PickObject(farPick);
            if (a_immediateWhenNoGround && !farPick.rayOutput.rootCollidable) {
                // player only: nothing within ~105 u below is a real drop (tall ledge), switch at once.
                // Other actors (enemies being juggled) keep the K=6 behaviour: a probe that hits nothing is not a drop.
                ResetMisses(a_controller);
                logger::info("[AMR-AIR] no ground within {:.0f} u -> in air", kFarProbeLength * 69.991249F);
                return true;
            }
            if (farPick.rayOutput.HasHit() && farPick.rayOutput.hitFraction * kFarProbeLength > kFarGroundDistance) {
                ResetMisses(a_controller);
                logger::info("[AMR-AIR] far probe: ground {:.1f} u below -> in air",
                    farPick.rayOutput.hitFraction * kFarProbeLength * 69.991249F);
                return true;
            }
            std::lock_guard guard(g_missLock);
            if (g_misses.size() > 256) {
                g_misses.clear();
            }
            const auto count = ++g_misses[a_controller];
            if (count >= (a_immediateWhenNoGround ? kMissesBeforeAirPlayer : kMissesBeforeAirOther)) {
                g_misses.erase(a_controller);
                logger::info("[AMR-AIR] {} consecutive ray misses -> in air", count);
                return true;
            }
            return false;
        }
    }

    RE::hkpCharacterStateType hkpCharacterContext::GetCharacterState_Hook(RE::hkpCharacterContext* a_this)
    {
        if (!a_this) {
            return RE::hkpCharacterStateType::kOnGround;
        }
        const auto state = amr::abi::GroundState(a_this);
        return !state || state->unk10 ? a_this->currentState : RE::hkpCharacterStateType::kSwimming;
    }

    void SimulateStatePhysics(RE::bhkCharacterStateOnGround* a_this, RE::bhkCharacterController* a_characterController)
    {
        if (!a_this || !a_characterController) {
            return;
        }
        const RE::NiPointer<RE::bhkCharacterController> controllerOwner(a_characterController);
        RE::NiPointer<RE::Actor> character;
        {
            RE::BSSpinLockGuard sourceGuard(a_characterController->lock);
            for (auto sink : a_characterController->sinks) {
                if (std::find(a_characterController->pendingUnregisters.begin(), a_characterController->pendingUnregisters.end(), sink) !=
                    a_characterController->pendingUnregisters.end()) {
                    continue;
                }
                if (const auto candidate = amr::abi::CharacterFromSink(sink)) {
                    character = candidate->GetHandle().get();
                    if (character) {
                        break;
                    }
                }
            }
        }
        if (const auto graph = amr::abi::ActiveGraph(character.get())) {
            RE::BSTEventSource<RE::BSAnimationGraphEvent>* source = graph.get();
            const auto event = amr::MakePhysicsUpdateEvent(character.get());
            source->SendEvent(&event);
        }
        // v6: the custom on-ground step below is only needed while an AMR motion clip drives this actor (or a
        // launch is being followed). Otherwise run the game's own on-ground physics, so ordinary walking and
        // standing behave exactly as without AMR (no brief switches to in-air on slopes and uneven ground).
        {
            const auto graph = amr::abi::ActiveGraph(character.get());
            const bool motion = graph && HasAnimMotionData(std::addressof(graph->characterInstance));
            float      velocity = 0.0F;
            float      targetZ = 0.0F;
            bool       logNow = false;
            const bool lifting = amr::vertical::Command(std::addressof(a_characterController->context),
                amr::vertical::ControllerHeight(a_characterController), velocity, targetZ, logNow);
            if (!motion && !lifting) {
                bhkCharacterStateOnGround::SimulateStatePhysics(a_this, a_characterController);
                return;
            }
        }

        const RE::NiPointer<RE::bhkWorld> world(a_characterController->GetHavokWorld());
        if (!world) {
            return;
        }
        RE::hkVector4 position;
        a_characterController->GetPosition(position, true);
        a_characterController->fallStartHeight = position.quad.m128_f32[2] * 69.991249F;
        RE::CFilter collisionFilterInfo;
        a_characterController->GetCollisionFilterInfo(collisionFilterInfo);
        RE::bhkPickData pickData;
        pickData.rayInput.from = position;
        pickData.rayInput.to = position - RE::hkVector4{ 0.0F, 0.0F, 0.5F, 0.0F };
        pickData.rayInput.filterInfo = collisionFilterInfo;
        world->PickObject(pickData);
        if (pickData.rayOutput.rootCollidable) {
            ResetMisses(a_characterController);
            if (pickData.rayOutput.normal.quad.m128_f32[2] < a_characterController->zNormal) {
                a_characterController->flags.set(RE::CHARACTER_FLAGS::kSupport);
            }
        } else if (ShouldGoInAir(a_characterController, world.get(), position, collisionFilterInfo,
                       character && character->IsPlayerRef())) {
            a_characterController->wantState = RE::hkpCharacterStateType::kInAir;
            a_characterController->flags.reset(RE::CHARACTER_FLAGS::kSupport);
        }
        RE::hkVector4 desiredVelocity = a_characterController->velocityMod;
        std::swap(desiredVelocity.quad.m128_f32[0], desiredVelocity.quad.m128_f32[1]);
        amr::abi::MovementInput movementUtil{};
        movementUtil.forward = a_characterController->forwardVec;
        movementUtil.up = RE::hkVector4{ 0.0F, 0.0F, 1.0F, 0.0F };
        movementUtil.surfaceNormal = a_characterController->supportNorm;
        movementUtil.currentVelocity = a_characterController->outVelocity;
        movementUtil.desiredVelocity = desiredVelocity;
        movementUtil.surfaceVelocity = a_characterController->surfaceInfo.surfaceVelocity;
        movementUtil.gain = 1.0F;
        movementUtil.maxVelocityDelta = 500.0F;
        static REL::Relocation<void (*)(const amr::abi::MovementInput&, RE::hkVector4&)> calculateMovement{ REL::ID(81002) };
        calculateMovement(movementUtil, a_characterController->outVelocity);
        a_characterController->outVelocity = a_characterController->outVelocity + a_characterController->surfaceInfo.surfaceVelocity;
        static REL::Relocation<const RE::hkVector4* (*)(const RE::bhkWorld*)> getGravity{ REL::ID(77838) };
        if (const auto gravity = getGravity(world.get())) {
            a_characterController->outVelocity = a_characterController->outVelocity + *gravity * a_characterController->stepInfo.deltaTime;
        }
        // Vertical root motion (VerticalMotion.h): a launch curve lifts the actor off the ground; the in-air
        // hook below keeps the actor on the curve.
        {
            const float currentZ = amr::vertical::Z(position) * amr::vertical::kHavokToGame;
            float       velocity = 0.0F;
            float       targetZ = 0.0F;
            bool        logNow = false;
            if (amr::vertical::Command(std::addressof(a_characterController->context), currentZ, velocity, targetZ, logNow) &&
                velocity > 50.0F) {
                amr::vertical::SetZ(a_characterController->outVelocity, velocity / amr::vertical::kHavokToGame);
                a_characterController->wantState = RE::hkpCharacterStateType::kInAir;
                a_characterController->flags.reset(RE::CHARACTER_FLAGS::kSupport);
                if (logNow) {
                    logger::info("[AMR-VZ] lift off: {:.1f} u/s, height {:.1f} -> curve {:.1f}", velocity, currentZ, targetZ);
                }
            }
        }
        if (a_characterController->wantState != RE::hkpCharacterStateType::kTotal) {
            a_characterController->context.currentState = a_characterController->wantState;
            a_characterController->wantState = RE::hkpCharacterStateType::kTotal;
        }
        const auto stateType = a_characterController->context.currentState;
        if (stateType != RE::hkpCharacterStateType::kOnGround) {
            const auto state = skyrim_cast<RE::bhkCharacterState*>(amr::abi::RegisteredState(&a_characterController->context, stateType));
            amr::abi::SimulateStatePhysics(state, a_characterController);
        }
    }

    void SimulateInAirPhysics(RE::bhkCharacterState* a_this, RE::bhkCharacterController* a_characterController)
    {
        bhkCharacterStateInAir::SimulateStatePhysics(a_this, a_characterController);
        if (!a_characterController) {
            return;
        }
        const float currentZ = amr::vertical::ControllerHeight(a_characterController);
        float       velocity = 0.0F;
        float       targetZ = 0.0F;
        bool        logNow = false;
        if (amr::vertical::Command(std::addressof(a_characterController->context), currentZ, velocity, targetZ, logNow)) {
            // stay on the launch curve (rise and hover); once it comes down gravity / CGAT take over (VerticalMotion.h)
            amr::vertical::SetZ(a_characterController->outVelocity, velocity / amr::vertical::kHavokToGame);
            if (logNow) {
                logger::info("[AMR-VZ] in air: {:.1f} u/s, height {:.1f} -> curve {:.1f}", velocity, currentZ, targetZ);
            }
        }
    }

    // Outermost on-ground step (installed late): other plugins' hooks run inside it; when a launch curve lifts the
    // actor, re-apply the lift speed they may have cleared.
    void SimulateGroundOuter(RE::bhkCharacterStateOnGround* a_this, RE::bhkCharacterController* a_characterController)
    {
        bhkCharacterStateOnGround::OuterSimulateStatePhysics(a_this, a_characterController);
        if (!a_characterController) {
            return;
        }
        const float currentZ = amr::vertical::ControllerHeight(a_characterController);
        float       velocity = 0.0F;
        float       targetZ = 0.0F;
        bool        logNow = false;
        if (amr::vertical::Command(std::addressof(a_characterController->context), currentZ, velocity, targetZ, logNow) &&
            velocity > 50.0F) {
            amr::vertical::SetZ(a_characterController->outVelocity, velocity / amr::vertical::kHavokToGame);
        }
    }
}
