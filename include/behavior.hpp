#ifndef BEHAVIOR_HPP_INCLUDED
#define BEHAVIOR_HPP_INCLUDED

enum class CommandMode { MANUAL, EXPLORATION, WALLFOLLOW };

class BehaviorManager {
public:
    BehaviorManager();

    // ID 0 -> MANUAL, 1 -> EXPLORATION, 42 -> WALLFOLLOW (autres IDs ignorés)
    // Retourne true si le mode a changé.
    bool setBehavior(int arucoID);
    CommandMode currentMode() const;

private:
    CommandMode commandMode_;
};

#endif // BEHAVIOR_HPP_INCLUDED
