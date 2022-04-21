// clang-format off
#include <iostream>
#include <opencv2/opencv.hpp>
#include "rasterizer.hpp"
#include "global.hpp"
#include "Triangle.hpp"

constexpr double MY_PI = 3.1415926;

Eigen::Matrix4f get_view_matrix(Eigen::Vector3f eye_pos)
{
    Eigen::Matrix4f view = Eigen::Matrix4f::Identity();

    Eigen::Matrix4f translate;
    translate << 1,0,0,-eye_pos[0],
                 0,1,0,-eye_pos[1],
                 0,0,1,-eye_pos[2],
                 0,0,0,1;

    view = translate*view;

    return view;
}

Eigen::Matrix4f get_model_matrix(float rotation_angle)
{
    Eigen::Matrix4f model = Eigen::Matrix4f::Identity();
    return model;
}

Eigen::Matrix4f get_projection_matrix(float eye_fov, float aspect_ratio, float zNear, float zFar)
{
    // TODO: Copy-paste your implementation from the previous assignment.
    // Eigen::Matrix4f projection;

    // return projection;

    // wyh copy-paste Assignment1
    // Students will implement this function
    // wyh projection = Mortho * Mpersp2ortho
    // wyh Mortho = Mr * Mt
    Eigen::Matrix4f projection = Eigen::Matrix4f::Identity();

    // TODO: Implement this function
    // Create the projection matrix for the given parameters.
    // Then return it.

    Eigen::Matrix4f Mpersp2ortho = Eigen::Matrix4f::Identity();
    Eigen::Matrix4f Mortho = Eigen::Matrix4f::Identity();
    Eigen::Matrix4f Mr = Eigen::Matrix4f::Identity();
    Eigen::Matrix4f Mt = Eigen::Matrix4f::Identity();

    Mr << 2/(2*((std::tan(eye_fov/2)/10)*aspect_ratio)), 0, 0, 0,
        0, 2/(2*(std::tan(eye_fov/2)/10)), 0, 0,
        0, 0, 2/(zNear-zFar), 0,
        0, 0, 0, 1;

    Mortho = Mr * Mt; // wyh Mortho: (正交)投影矩阵 归一化正交投影矩阵 缩放 * 平移 (此题位于中心, 无需平移, 即Mt为I)

    Mpersp2ortho << zNear, 0, 0, 0, // wyh Mpersp2ortho: (透视->正交)变换矩阵
        0, zNear, 0, 0,
        0, 0, zNear+zFar, -zNear*zFar,
        0, 0, 1, 0;

    projection = Mortho * Mpersp2ortho; // wyh (透视)投影矩阵 = (正交)投影矩阵 * (透视->正交)变换矩阵
    // projection = Mpersp2ortho; // wyh 若用(正交)投影矩阵当作(透视)投影矩阵, 结果是不对的
    // std::cout << "wyh projection" << std::endl;

    return projection;
}

int main(int argc, const char** argv)
{
    float angle = 0;
    bool command_line = false;
    std::string filename = "output.png";

    if (argc == 2)
    {
        command_line = true;
        filename = std::string(argv[1]);
    }

    rst::rasterizer r(700, 700);

    Eigen::Vector3f eye_pos = {0,0,5};
    // wyh eye_pos表示的是摄像机在世界空间所做的变换, 需要平移整个观察空间, 让摄像机原点位于世界坐标原点(反向思维好计算)

    std::vector<Eigen::Vector3f> pos // wyh 作业2共两个三角形, 所以6个顶点
            {
                    {2, 0, -2},
                    {0, 2, -2},
                    {-2, 0, -2},
                    {3.5, -1, -5},
                    {2.5, 1.5, -5},
                    {-1, 0.5, -5}
            };

    std::vector<Eigen::Vector3i> ind // wyh 两个三角形的索引
            {
                    {0, 1, 2},
                    {3, 4, 5}
            };

    std::vector<Eigen::Vector3f> cols // wyh 两个三角形顶点的color
            {
                    {217.0, 238.0, 185.0},
                    {217.0, 238.0, 185.0},
                    {217.0, 238.0, 185.0}, // 3个顶点颜色一样
                    {185.0, 217.0, 238.0},
                    {185.0, 217.0, 238.0},
                    {185.0, 217.0, 238.0}
            };

    auto pos_id = r.load_positions(pos);
    auto ind_id = r.load_indices(ind);
    auto col_id = r.load_colors(cols);

    int key = 0;
    int frame_count = 0;

    if (command_line)
    {
        r.clear(rst::Buffers::Color | rst::Buffers::Depth);

        r.set_model(get_model_matrix(angle));
        r.set_view(get_view_matrix(eye_pos));
        r.set_projection(get_projection_matrix(45, 1, 0.1, 50));

        r.draw(pos_id, ind_id, col_id, rst::Primitive::Triangle); // wyh 比作业1多了col_id
        cv::Mat image(700, 700, CV_32FC3, r.frame_buffer().data());
        image.convertTo(image, CV_8UC3, 1.0f);
        cv::cvtColor(image, image, cv::COLOR_RGB2BGR);

        cv::imwrite(filename, image);

        return 0;
    }

    while(key != 27)
    {
        r.clear(rst::Buffers::Color | rst::Buffers::Depth);

        r.set_model(get_model_matrix(angle));
        r.set_view(get_view_matrix(eye_pos));
        r.set_projection(get_projection_matrix(45, 1, 0.1, 50));

        r.draw(pos_id, ind_id, col_id, rst::Primitive::Triangle);

        cv::Mat image(700, 700, CV_32FC3, r.frame_buffer().data());
        image.convertTo(image, CV_8UC3, 1.0f);
        cv::cvtColor(image, image, cv::COLOR_RGB2BGR);
        cv::imshow("image", image);
        key = cv::waitKey(10);

        std::cout << "frame count: " << frame_count++ << '\n';
    }

    return 0;
}
// clang-format on