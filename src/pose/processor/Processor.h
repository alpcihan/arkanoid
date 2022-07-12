#pragma once

#include "pose/types.h"
#include "processor-chain/ProcessorChain.h"

namespace pose
{
    template <class Input, class Output>
    class Processor : public ProcessorLink
    {
    public:
        const Output &getLastResult() { return result; }

    protected:
        virtual void process(const Input &input, Output* output) = 0;

    protected:
        Output result;

    private:
        void *process(void *in) override;
    };

    template <class Input, class Output>
    void *Processor<Input, Output>::process(void *in)
    {
        process(*(Input *)in, nullptr);
        return (void *)&getLastResult();
    }
}