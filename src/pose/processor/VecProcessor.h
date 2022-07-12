#pragma once

#include "Processor.h"

namespace pose
{
    template <class P, class In, class Out>
    class VecProcessor : public Processor<std::vector<In>, std::vector<Out>>
    {
    public:
        void process(const std::vector<In> &input, std::vector<Out> *output) override;
        P &getProcessor();

    private:
        P processor;
    };

    template <class P, class In, class Out>
    void VecProcessor<P, In, Out>::process(const std::vector<In> &input, std::vector<Out> *output)
    {
        this->result.clear();

        for (auto &row : input)
        {
            processor.process(row, nullptr);
            this->result.push_back(std::move(processor.getLastResult()));
        }
    }

    template <class P, class In, class Out>
    P &VecProcessor<P, In, Out>::getProcessor()
    {
        return processor;
    }
}