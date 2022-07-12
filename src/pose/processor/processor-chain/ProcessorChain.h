#pragma once

#include "pose/types.h"
#include <unordered_map>

namespace pose
{
    class ProcessorLink
    {
    protected:
        virtual void *process(void *in) = 0;

        friend class ProcessorChain;
    };

    class ProcessorChain
    {
    public:
        void add(ProcessorLink &processor, const std::string& key = "");
        void process(void *input) const;

        template <class T>
        T& getProcessor(const std::string& key) const;

    private:
        std::vector<ProcessorLink *> chain;
        std::unordered_map<std::string, ProcessorLink*> keyToProcessor;
    };

    template <class T>
    T& ProcessorChain::getProcessor(const std::string& key) const
    {
       ProcessorLink* processor = keyToProcessor.at(key);

       return *(T*)processor;
    }
}
