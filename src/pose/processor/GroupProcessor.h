#pragma once

#include "Processor.h"

namespace pose
{
    template <class P, class In, class Out>
    class GroupProcessor : public Processor<std::vector<std::vector<In>>, std::vector<std::vector<Out>>>
    {
    public:
        void process(const std::vector<std::vector<In>> &input, std::vector<std::vector<Out>> *output) override;
        P &getProcessor();

    private:
        P processor;
    };

    template <class P, class In, class Out>
    void GroupProcessor<P, In, Out>::process(const std::vector<std::vector<In>> &input, std::vector<std::vector<Out>> *output)
    {
        this->result.clear();

        for (auto &row : input)
        {
            std::vector<Out> temp;
            for (auto &column : row)
            {
                processor.process(column, nullptr);
                temp.push_back(std::move(processor.getLastResult()));
            }
            this->result.push_back(std::move(temp));
        }
    }

    template <class P, class In, class Out>
    P &GroupProcessor<P, In, Out>::getProcessor()
    {
        return processor;
    }
}