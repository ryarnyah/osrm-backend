module.exports = {
    default: '--strict --tags ~@stress --tags ~@todo --tags ~@mld-only --tags ~@restriction-way --require features/support --require features/step_definitions',
    verify: '--strict --tags ~@stress --tags ~@todo --tags ~@mld-only --tags ~@restriction-way -f progress --require features/support --require features/step_definitions',
    todo: '--strict --tags @todo --require features/support --require features/step_definitions',
    all: '--strict --require features/support --require features/step_definitions',
    mld: '--strict --tags ~@stress --tags ~@todo --tags ~@restriction-way --require features/support --require features/step_definitions -f progress'
};
