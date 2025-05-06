#include <bananas_aruco/concrete_board.h>

#include <stdexcept>
#include <string>

#include <nlohmann/json.hpp>

#include <bananas_aruco/box_board.h>
#include <bananas_aruco/grid_board.h>

namespace board {

void from_json(const nlohmann::json &j, ConcreteBoard &board) {
    const auto type{j.at("type").get<std::string>()};
    if (type == "box") {
        board = j.at("settings").get<BoxSettings>();
    } else if (type == "grid") {
        board = j.at("settings").get<GridSettings>();
    } else {
        throw std::runtime_error{"Bad static environment member type " + type};
    }
}

} // namespace board
