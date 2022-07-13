#include "helpers.h"

namespace help
{
    std::shared_ptr<gfx::Texture> getBGTexture(const pose::Image &frame)
    {
        pose::Image bgImage;
        cv::flip(frame, bgImage, 0);
        cv::cvtColor(bgImage, bgImage, cv::COLOR_BGR2RGB);
        return std::make_shared<gfx::Texture>(bgImage.ptr(), bgImage.cols, bgImage.rows, bgImage.channels());
    }

    void printMat4(const glm::mat4& mtrx)
    {
        std::cout << "Matrix4: " << std::endl;
        for(int i = 0; i < 4; i++)
        {
            for(int j = 0; j < 4; j++)
            {
                std::cout << mtrx[j][i] << " ";
            }
            std::cout << std::endl;
        }

        std::cout << std::endl;
    }
}