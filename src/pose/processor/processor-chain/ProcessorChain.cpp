#include "ProcessorChain.h"

namespace pose
{
    void ProcessorChain::add(ProcessorLink &processor, const std::string& key)
    {
        if(key != "")
            keyToProcessor[key] = &processor;

        chain.push_back(&processor);
    }

    void ProcessorChain::process(void *input) const
    {
        void *temp = input;

        for (auto &processor : chain)
        {
            temp = processor->process(temp);
        }
    }
}