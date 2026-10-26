#include "behavior.hpp"

BehaviorManager::BehaviorManager() : commandMode_(CommandMode::MANUAL) {}

bool BehaviorManager::setBehavior(int arucoID) {
    CommandMode m = commandMode_;
    if      (arucoID == 0)  m = CommandMode::MANUAL;
    else if (arucoID == 1)  m = CommandMode::EXPLORATION;
    else if (arucoID == 42) m = CommandMode::WALLFOLLOW;
    bool changed = (m != commandMode_);
    commandMode_ = m;
    return changed;
}

CommandMode BehaviorManager::currentMode() const { return commandMode_; }
