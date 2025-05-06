#ifndef CONCRETE_BOARD_H_
#define CONCRETE_BOARD_H_

#include <variant>

#include <nlohmann/json_fwd.hpp>

#include <bananas_aruco/box_board.h>
#include <bananas_aruco/grid_board.h>

namespace board {

using ConcreteBoard = std::variant<BoxSettings, GridSettings>;
void from_json(const nlohmann::json &j, ConcreteBoard &board);

} // namespace board

#endif // CONCRETE_BOARD_H_
