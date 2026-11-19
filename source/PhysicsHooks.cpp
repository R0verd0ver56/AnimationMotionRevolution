#include "Hooks.h"
#include "PhysicsEvents.h"
#include "VerticalMotion.h"
#include "utils/Logger.h"

#include <algorithm>

namespace hooks
{
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
            if (pickData.rayOutput.normal.quad.m128_f32[2] < a_characterController->zNormal) {
                a_characterController->flags.set(RE::CHARACTER_FLAGS::kSupport);
            }
        } else {
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
        // Vertical root motion (VerticalMotion.h): a rising animmotion curve lifts the actor off the
        // ground; the in-air hook below keeps following the curve.
        {
            float wantSpeed = 0.0F;
            bool  logNow = false;
            if (amr::vertical::Speed(std::addressof(a_characterController->context), wantSpeed, logNow) &&
                wantSpeed > amr::vertical::kMinLiftSpeed) {
                a_characterController->outVelocity.quad.m128_f32[2] = wantSpeed / amr::vertical::kHavokToGame;
                a_characterController->wantState = RE::hkpCharacterStateType::kInAir;
                a_characterController->flags.reset(RE::CHARACTER_FLAGS::kSupport);
                if (logNow) {
                    logger::info("[AMR-VZ] lift off: curve {:.1f} u/s at height {:.1f}", wantSpeed,
                        position.quad.m128_f32[2] * amr::vertical::kHavokToGame);
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
        float wantSpeed = 0.0F;
        bool  logNow = false;
        if (amr::vertical::Speed(std::addressof(a_characterController->context), wantSpeed, logNow)) {
            // keep following the rising curve in the air; after the peak gravity / CGAT take over (VerticalMotion.h)
            a_characterController->outVelocity.quad.m128_f32[2] = wantSpeed / amr::vertical::kHavokToGame;
            if (logNow) {
                RE::hkVector4 position;
                a_characterController->GetPosition(position, true);
                logger::info("[AMR-VZ] in air: curve {:.1f} u/s at height {:.1f}", wantSpeed,
                    position.quad.m128_f32[2] * amr::vertical::kHavokToGame);
            }
        }
    }

    // Outermost on-ground step (installed late): other plugins' hooks run inside it; when this frame's animmotion curve
    // lifts the actor, re-apply the lift speed they may have cleared.
    void SimulateGroundOuter(RE::bhkCharacterStateOnGround* a_this, RE::bhkCharacterController* a_characterController)
    {
        bhkCharacterStateOnGround::OuterSimulateStatePhysics(a_this, a_characterController);
        if (!a_characterController) {
            return;
        }
        float wantSpeed = 0.0F;
        bool  logNow = false;
        if (amr::vertical::Speed(std::addressof(a_characterController->context), wantSpeed, logNow) &&
            wantSpeed > amr::vertical::kMinLiftSpeed) {
            a_characterController->outVelocity.quad.m128_f32[2] = wantSpeed / amr::vertical::kHavokToGame;
        }
    }
}
