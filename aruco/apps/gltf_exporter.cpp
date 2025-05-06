#include <algorithm>
#include <array>
#include <cassert>
#include <cmath>
#include <cstddef>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <ios>
#include <iostream>
#include <limits>
#include <numeric>
#include <string>
#include <variant>
#include <vector>

#include <gsl/span>

#include <opencv2/core/mat.hpp>
#include <opencv2/core/matx.hpp>
#include <opencv2/core/types.hpp>
#include <opencv2/core/utility.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/objdetect/aruco_board.hpp>
#include <opencv2/objdetect/aruco_dictionary.hpp>

#include <nlohmann/json.hpp>

#define TINYGLTF_IMPLEMENTATION
#define TINYGLTF_NO_STB_IMAGE
#define TINYGLTF_NO_STB_IMAGE_WRITE
#define TINYGLTF_NO_INCLUDE_JSON
#include <tiny_gltf.h>

#include <tinyxml2.h>

#include <bananas_aruco/board.h>
#include <bananas_aruco/box_board.h>
#include <bananas_aruco/concrete_board.h>
#include <bananas_aruco/grid_board.h>

namespace {

const char *const about{
    "Generate binary glTF and SDF files from a set of ArUco marker placements"};
const char *const keys{
    "{@inpath  | <none> | JSON file containing box descriptions }"
    "{o        | .      | Output directory }"
    "{sdf      |        | Whether to generate SDF files for Gazebo }"};

constexpr std::size_t corners_per_marker{4};
constexpr std::size_t floats_per_position{3};
constexpr std::size_t floats_per_texcoord{2};

/// Generate glTF position and texcoord array data for the given marker corners.
void fill_marker_data(
    gsl::span<const cv::Point3f, corners_per_marker> corners,
    gsl::span<float, corners_per_marker * floats_per_position> positions,
    gsl::span<float, corners_per_marker * floats_per_texcoord> texcoords) {
    std::size_t out_index{0};
    // Go counterclockwise from bottom left to get the winding order correct.
    for (auto corner = corners.rbegin(); corner != corners.rend(); ++corner) {
        positions[out_index++] = corner->x;
        positions[out_index++] = corner->y;
        positions[out_index++] = corner->z;
    }

    constexpr std::array<const float, corners_per_marker * floats_per_texcoord>
        texture_coordinates = {
            0.0F, 1.0F, // Bottom left
            1.0F, 1.0F, // Bottom right
            1.0F, 0.0F, // Top right
            0.0F, 0.0F, // Top left
        };
    std::copy(texture_coordinates.cbegin(), texture_coordinates.cend(),
              texcoords.begin());
}

/// Produce a glTF asset for the given ArUco board.
auto produce_board_model(const cv::aruco::Board &board) -> tinygltf::Model {
    tinygltf::Model model{};
    const auto &markers{board.getObjPoints()};

    const std::size_t num_position_floats{markers.size() * corners_per_marker *
                                          floats_per_position};
    const std::size_t num_texcoord_floats{markers.size() * corners_per_marker *
                                          floats_per_texcoord};
    std::vector<float> geometry_data(num_position_floats + num_texcoord_floats);
    const gsl::span<float> positions{
        gsl::span{geometry_data}.subspan(0, num_position_floats)};
    const gsl::span<float> texcoords{gsl::span{geometry_data}.subspan(
        num_position_floats, num_texcoord_floats)};

    std::size_t position_index{0};
    std::size_t texcoord_index{0};
    for (const auto &marker : markers) {
        fill_marker_data(
            gsl::span<const cv::Point3f, 4>{marker},
            gsl::span<float, corners_per_marker * floats_per_position>{
                positions.subspan(position_index,
                                  corners_per_marker * floats_per_position)},
            gsl::span<float, corners_per_marker * floats_per_texcoord>{
                texcoords.subspan(texcoord_index,
                                  corners_per_marker * floats_per_texcoord)});
        position_index += corners_per_marker * floats_per_position;
        texcoord_index += corners_per_marker * floats_per_texcoord;
    }

    // NOLINTBEGIN(cppcoreguidelines-pro-type-reinterpret-cast)
    auto &buffer{model.buffers.emplace_back()};
    buffer.data = std::vector<unsigned char>(
        reinterpret_cast<const unsigned char *>(geometry_data.data()),
        reinterpret_cast<const unsigned char *>(geometry_data.data() +
                                                geometry_data.size()));
    // NOLINTEND(cppcoreguidelines-pro-type-reinterpret-cast)

    model.bufferViews.reserve(2 + markers.size());
    auto &positions_view{model.bufferViews.emplace_back()};
    positions_view.byteOffset = 0;
    positions_view.byteLength = positions.size_bytes();
    positions_view.byteStride = floats_per_position * sizeof(float);
    positions_view.target = TINYGLTF_TARGET_ARRAY_BUFFER;
    positions_view.buffer = 0;

    auto &texcoords_view{model.bufferViews.emplace_back()};
    texcoords_view.byteOffset = positions.size_bytes();
    texcoords_view.byteLength = texcoords.size_bytes();
    texcoords_view.byteStride = floats_per_texcoord * sizeof(float);
    texcoords_view.target = TINYGLTF_TARGET_ARRAY_BUFFER;
    texcoords_view.buffer = 0;

    auto &sampler{model.samplers.emplace_back()};
    sampler.wrapS = TINYGLTF_TEXTURE_WRAP_CLAMP_TO_EDGE;
    sampler.wrapT = TINYGLTF_TEXTURE_WRAP_CLAMP_TO_EDGE;
    sampler.minFilter = TINYGLTF_TEXTURE_FILTER_NEAREST;
    sampler.magFilter = TINYGLTF_TEXTURE_FILTER_NEAREST;

    for (const auto id : board.getIds()) {
        auto &image_view{model.bufferViews.emplace_back()};
        image_view.byteOffset = buffer.data.size();
        image_view.buffer = 0;

        cv::Mat image;
        // Gazebo doesn't support texture filtering modes. Give it some extra
        // pixels to work with.
        constexpr int pixel_factor{16};
        board.getDictionary().generateImageMarker(
            id, (board.getDictionary().markerSize + 2) * pixel_factor, image);

        std::vector<unsigned char> image_png;
        cv::imencode(".png", image, image_png);

        buffer.data.insert(buffer.data.end(), image_png.cbegin(),
                           image_png.cend());
        image_view.byteLength = image_png.size();

        auto &gltf_image{model.images.emplace_back()};
        gltf_image.bufferView = static_cast<int>(model.bufferViews.size()) - 1;
        gltf_image.mimeType = "image/png";

        auto &texture{model.textures.emplace_back()};
        texture.source = static_cast<int>(model.images.size()) - 1;
        texture.sampler = 0;
    }

    constexpr std::size_t accessors_per_marker{2};
    model.accessors.resize(markers.size() * accessors_per_marker);
    model.meshes.resize(markers.size());
    model.materials.resize(markers.size());

    for (int marker_index{0}; marker_index < static_cast<int>(markers.size());
         ++marker_index) {
        const std::size_t base_accessor_index{marker_index *
                                              accessors_per_marker};
        const std::size_t position_accessor_index{base_accessor_index + 0};
        const std::size_t texcoord_accessor_index{base_accessor_index + 1};

        auto &position_accessor{model.accessors[position_accessor_index]};
        position_accessor.byteOffset = marker_index * corners_per_marker *
                                       floats_per_position * sizeof(float);
        position_accessor.count = corners_per_marker;
        position_accessor.type = TINYGLTF_TYPE_VEC3;
        position_accessor.componentType = TINYGLTF_COMPONENT_TYPE_FLOAT;
        position_accessor.bufferView = 0;

        const auto corner_positions{
            positions.subspan(static_cast<size_t>(marker_index) *
                                  corners_per_marker * floats_per_position,
                              floats_per_position * corners_per_marker)};
        position_accessor.minValues.resize(
            floats_per_position, std::numeric_limits<double>::infinity());
        position_accessor.maxValues.resize(
            floats_per_position, -std::numeric_limits<double>::infinity());
        for (std::size_t i = 0; i < corner_positions.size(); ++i) {
            position_accessor.minValues[i % floats_per_position] =
                std::min(position_accessor.minValues[i % floats_per_position],
                         double{corner_positions[i]});
            position_accessor.maxValues[i % floats_per_position] =
                std::max(position_accessor.maxValues[i % floats_per_position],
                         double{corner_positions[i]});
        }

        auto &texcoord_accessor{model.accessors[texcoord_accessor_index]};
        texcoord_accessor.byteOffset = marker_index * corners_per_marker *
                                       floats_per_texcoord * sizeof(float);
        texcoord_accessor.count = corners_per_marker;
        texcoord_accessor.type = TINYGLTF_TYPE_VEC2;
        texcoord_accessor.componentType = TINYGLTF_COMPONENT_TYPE_FLOAT;
        texcoord_accessor.bufferView = 1;

        auto &material{model.materials[marker_index]};
        material.pbrMetallicRoughness.baseColorTexture.index = marker_index;
        material.pbrMetallicRoughness.baseColorTexture.texCoord = 0;
        // NOTE: Gazebo requires all materials to have different names.
        material.name = "material" + std::to_string(marker_index);

        auto &primitive{model.meshes[marker_index].primitives.emplace_back()};
        primitive.attributes.emplace("POSITION", position_accessor_index);
        primitive.attributes.emplace("TEXCOORD_0", texcoord_accessor_index);
        primitive.mode = TINYGLTF_MODE_TRIANGLE_FAN;
        primitive.material = marker_index;
    }

    model.nodes.resize(markers.size());
    int marker_index{0};
    for (auto &node : model.nodes) {
        node.mesh = marker_index++;
    }

    auto &scene = model.scenes.emplace_back();
    scene.nodes.resize(markers.size());
    std::iota(scene.nodes.begin(), scene.nodes.end(), 0);
    model.defaultScene = 0;

    return model;
}

constexpr std::size_t box_faces{6};
constexpr std::size_t box_corners_per_face{4};
constexpr std::array<float,
                     box_faces * box_corners_per_face * floats_per_position>
    cube_vertices{
        // Forward
        -0.5F, -0.5F, +0.5F, //
        +0.5F, -0.5F, +0.5F, //
        +0.5F, +0.5F, +0.5F, //
        -0.5F, +0.5F, +0.5F, //
        // Backward
        +0.5F, -0.5F, -0.5F, //
        -0.5F, -0.5F, -0.5F, //
        -0.5F, +0.5F, -0.5F, //
        +0.5F, +0.5F, -0.5F, //
        // Left
        +0.5F, -0.5F, +0.5F, //
        +0.5F, -0.5F, -0.5F, //
        +0.5F, +0.5F, -0.5F, //
        +0.5F, +0.5F, +0.5F, //
        // Right
        -0.5F, -0.5F, -0.5F, //
        -0.5F, -0.5F, +0.5F, //
        -0.5F, +0.5F, +0.5F, //
        -0.5F, +0.5F, -0.5F, //
        // Up
        -0.5F, +0.5F, +0.5F, //
        +0.5F, +0.5F, +0.5F, //
        +0.5F, +0.5F, -0.5F, //
        -0.5F, +0.5F, -0.5F, //
        // Down
        -0.5F, -0.5F, -0.5F, //
        +0.5F, -0.5F, -0.5F, //
        +0.5F, -0.5F, +0.5F, //
        -0.5F, -0.5F, +0.5F, //
    };

/// Produce a glTF asset for the given box, including the markers.
auto produce_board(const cv::aruco::Dictionary &dictionary,
                   const board::BoxSettings &box) -> tinygltf::Model {
    // 1. Add in the ArUco markers
    const cv::aruco::Board board{
        board::to_cv(dictionary, board::make_board(box))};
    auto model{produce_board_model(board)};

    // 2. Add the box in afterwards
    assert(!model.buffers.empty());
    auto &buffer{model.buffers[0]};

    const std::size_t start_offset{(buffer.data.size() + sizeof(float) - 1) /
                                   sizeof(float) * sizeof(float)};
    const std::size_t cube_bytes{cube_vertices.size() * sizeof(float)};
    auto &positions_view{model.bufferViews.emplace_back()};
    positions_view.byteOffset = start_offset;
    positions_view.byteLength = cube_bytes;
    positions_view.byteStride = floats_per_position * sizeof(float);
    positions_view.target = TINYGLTF_TARGET_ARRAY_BUFFER;
    positions_view.buffer = 0;
    buffer.data.resize(start_offset + cube_bytes);

    // Make the box a bit smaller to avoid Z-fighting with the markers. The
    // markers are the real important part, the box is just visual extra.
    constexpr float size_factor{0.995F};
    const cv::Vec3f scaled_size{
        cv::Vec3f{box.size.width, box.size.height, box.size.depth} *
        size_factor};

    std::array<float, box_faces * box_corners_per_face * floats_per_position>
        vertices(cube_vertices);
    for (std::size_t vertex{0}; vertex < box_faces * box_corners_per_face;
         ++vertex) {
        vertices[(floats_per_position * vertex) + 0] *= scaled_size[0];
        vertices[(floats_per_position * vertex) + 1] *= scaled_size[1];
        vertices[(floats_per_position * vertex) + 2] *= scaled_size[2];
    }

    // NOLINTBEGIN(cppcoreguidelines-pro-type-reinterpret-cast)
    std::copy_n(reinterpret_cast<const unsigned char *>(vertices.data()),
                vertices.size() * sizeof(float),
                buffer.data.begin() +
                    static_cast<std::ptrdiff_t>(start_offset));
    // NOLINTEND(cppcoreguidelines-pro-type-reinterpret-cast)

    auto &material{model.materials.emplace_back()};
    // Cardboard brown, #a58855
    material.pbrMetallicRoughness.baseColorFactor = {0.37626, 0.24620, 0.09084,
                                                     1.0};

    auto &box_mesh{model.meshes.emplace_back()};
    for (std::size_t face{0}; face < box_faces; ++face) {
        auto &position_accessor{model.accessors.emplace_back()};
        position_accessor.byteOffset =
            face * box_corners_per_face * floats_per_position * sizeof(float);
        position_accessor.count = box_corners_per_face;
        position_accessor.type = TINYGLTF_TYPE_VEC3;
        position_accessor.componentType = TINYGLTF_COMPONENT_TYPE_FLOAT;
        position_accessor.bufferView =
            static_cast<int>(model.bufferViews.size()) - 1;
        position_accessor.minValues = {
            face == 2 ? 0.5F * scaled_size[0] : -0.5F * scaled_size[0],
            face == 4 ? 0.5F * scaled_size[1] : -0.5F * scaled_size[1],
            face == 0 ? 0.5F * scaled_size[2] : -0.5F * scaled_size[2]};
        position_accessor.maxValues = {
            face == 3 ? -0.5F * scaled_size[0] : 0.5F * scaled_size[0],
            face == 5 ? -0.5F * scaled_size[1] : 0.5F * scaled_size[1],
            face == 1 ? -0.5F * scaled_size[2] : 0.5F * scaled_size[2]};

        auto &primitive{box_mesh.primitives.emplace_back()};
        primitive.attributes.emplace("POSITION", model.accessors.size() - 1);
        primitive.mode = TINYGLTF_MODE_TRIANGLE_FAN;
        primitive.material = static_cast<int>(model.materials.size()) - 1;
    }

    auto &box_node{model.nodes.emplace_back()};
    box_node.mesh = static_cast<int>(model.meshes.size()) - 1;

    model.scenes[0].nodes.push_back(static_cast<int>(model.nodes.size()) - 1);

    return model;
}

auto produce_board(const cv::aruco::Dictionary &dictionary,
                   const board::GridSettings &grid) -> tinygltf::Model {
    return produce_board_model(
        board::to_cv(dictionary, board::make_board(grid)));
}

void produce_sdf_model_extras(tinyxml2::XMLPrinter & /*printer*/,
                              const board::BoxSettings & /*box*/) {}

void produce_sdf_model_extras(tinyxml2::XMLPrinter &printer,
                              const board::GridSettings & /*grid*/) {
    printer.OpenElement("static");
    printer.PushText("true");
    printer.CloseElement(); // </static>
}

void produce_sdf_link_extras(tinyxml2::XMLPrinter &printer,
                             const board::BoxSettings &box) {
    {
        printer.OpenElement("collision");
        printer.PushAttribute("name", "collision");
        {
            printer.OpenElement("density");
            // TODO(vainiovano): Select a proper density.
            printer.PushText("10.0");
            printer.CloseElement(); // </density>
        }
        {
            printer.OpenElement("geometry");
            printer.OpenElement("box");
            printer.OpenElement("size");
            printer.PushText(box.size.width);
            printer.PushText(" ");
            printer.PushText(box.size.height);
            printer.PushText(" ");
            printer.PushText(box.size.depth);
            printer.CloseElement(); // </size>
            printer.CloseElement(); // </box>
            printer.CloseElement(); // </geometry>
        }
        printer.CloseElement(); // </collision>
    }
    {
        printer.OpenElement("inertial");
        printer.PushAttribute("auto", true);
        printer.CloseElement(); // </inertial>
    }
}

void produce_sdf_link_extras(tinyxml2::XMLPrinter & /*printer*/,
                             const board::GridSettings & /*grid*/) {}

void produce_sdf(std::ostream &out, const std::string &name,
                 const std::filesystem::path &gltf_path,
                 const board::ConcreteBoard &board) {
    tinyxml2::XMLPrinter printer{};
    printer.PushHeader(false, true);

    printer.OpenElement("sdf");
    // SDFormat 1.11 (Gazebo Harmonic) added support for automatically computed
    // inertia.
    printer.PushAttribute("version", "1.11");

    printer.OpenElement("model");
    printer.PushAttribute("name", name.c_str());
    std::visit(
        [&printer](const auto &board) {
            produce_sdf_model_extras(printer, board);
        },
        board);
    printer.OpenElement("link");
    printer.PushAttribute("name", "link");
    {
        printer.OpenElement("pose");
        printer.PushAttribute("degrees", true);
        // Gazebo's glTF importer does not convert the coordinate system
        // correctly. In the Gazebo coordinate system [1], +X is forward, +Y is
        // left and +Z is up, so fix up our model pose to match it.
        //
        // [1] https://gazebosim.org/api/sim/8/frame_reference.html
        printer.PushText("0 0 0 90 0 90");
        printer.CloseElement(); // </pose>
    }
    {
        printer.OpenElement("visual");
        printer.PushAttribute("name", "visual");
        {
            printer.OpenElement("geometry");
            printer.OpenElement("mesh");
            printer.OpenElement("uri");
            printer.PushText(
                (std::string{"model://"} + gltf_path.string()).c_str());
            printer.CloseElement(); // </uri>
            printer.CloseElement(); // </mesh>
            printer.CloseElement(); // </geometry>
        }
        printer.CloseElement(); // </visual>
    }
    std::visit(
        [&printer](const auto &board) {
            produce_sdf_link_extras(printer, board);
        },
        board);
    printer.CloseElement(); // </link>
    printer.CloseElement(); // </model>
    printer.CloseElement(); // </sdf>

    out.write(printer.CStr(),
              static_cast<std::streamsize>(printer.CStrSize() - 1));
}

void produce_failed_open_diagnostics(std::ostream &stream,
                                     const std::filesystem::path &out_dir,
                                     const std::filesystem::path &out_path) {
    stream << "Failed to open output file "
           << std::quoted(out_path.string(), '`') << '\n';
    // Racy, but hopefully fine for error checking.
    if (!std::filesystem::exists(out_dir)) {
        stream << "Note: directory " << std::quoted(out_dir.string(), '`')
               << " does not exist.\n";
    } else if (!std::filesystem::is_directory(out_dir)) {
        stream << "Note: " << std::quoted(out_dir.string(), '`')
               << " is not a directory.\n";
    }
}

} // namespace

auto main(int argc, char *argv[]) -> int {
    cv::CommandLineParser parser{argc, argv, keys};
    parser.about(about);

    const auto in_path{parser.get<std::string>(0)};
    const std::filesystem::path out_dir{parser.get<std::string>("o")};
    const auto want_sdf{parser.has("sdf")};
    if (!parser.check()) {
        parser.printErrors();
        parser.printMessage();
        return EXIT_FAILURE;
    }

    const cv::aruco::Dictionary dictionary{cv::aruco::getPredefinedDictionary(
        cv::aruco::PredefinedDictionaryType::DICT_5X5_100)};

    std::vector<board::ConcreteBoard> board_settings;
    {
        std::ifstream in_stream{in_path};
        if (!in_stream) {
            std::cerr << "Failed to open input file\n";
            return EXIT_FAILURE;
        }

        try {
            const auto json = nlohmann::json::parse(in_stream);
            json.get_to(board_settings);
        } catch (const nlohmann::json::exception &e) {
            std::cerr << "Failed to parse board description file: " << e.what()
                      << '\n';
            return EXIT_FAILURE;
        }
    }

    for (std::size_t i{0}; i < board_settings.size(); ++i) {
        const std::string out_stem{std::string{"board_"} + std::to_string(i)};
        const std::string gltf_filename{out_stem + std::string{".glb"}};
        const std::filesystem::path gltf_out_path{out_dir / gltf_filename};
        const auto model{std::visit(
            [&dictionary](const auto &board) {
                return produce_board(dictionary, board);
            },
            board_settings[i])};

        {
            std::ofstream gltf_out{gltf_out_path,
                                   std::ios_base::out | std::ios_base::binary};
            if (!gltf_out) {
                produce_failed_open_diagnostics(std::cerr, out_dir,
                                                gltf_out_path);
                return EXIT_FAILURE;
            }

            try {
                tinygltf::TinyGLTF gltf{};
                gltf.WriteGltfSceneToStream(&model, gltf_out, false, true);
            } catch (const nlohmann::json::exception &e) {
                std::cerr << "Failed to write JSON output to "
                          << std::quoted(gltf_out_path.string(), '`') << ": "
                          << e.what() << '\n';
                return EXIT_FAILURE;
            }

            gltf_out.close();
            if (!gltf_out) {
                std::cerr << "Failed to write to glTF file "
                          << std::quoted(gltf_out_path.string(), '`') << '\n';
                return EXIT_FAILURE;
            }
            std::cerr << "Wrote " << std::quoted(gltf_out_path.string(), '`')
                      << '\n';
        }
        if (want_sdf) {
            const std::string sdf_filename{out_stem + std::string{".sdf"}};
            const std::filesystem::path sdf_out_path{out_dir / sdf_filename};
            std::ofstream sdf_out{sdf_out_path, std::ios_base::out};
            if (!sdf_out) {
                produce_failed_open_diagnostics(std::cerr, out_dir,
                                                sdf_out_path);
                return EXIT_FAILURE;
            }

            produce_sdf(sdf_out, out_stem, gltf_out_path, board_settings[i]);

            sdf_out.close();
            if (!sdf_out) {
                std::cerr << "Failed to write to SDF file "
                          << std::quoted(sdf_out_path.string(), '`') << '\n';
                return EXIT_FAILURE;
            }
            std::cerr << "Wrote " << std::quoted(sdf_out_path.string(), '`')
                      << '\n';
        }
    }
}
