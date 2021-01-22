#include <OWEN/DiscreteOutputMU110.hpp>

#include <iostream>
#include <thread>

auto main(int argc, char** argv) -> int32_t
{
   auto communicationOptions = OWEN::DiscreteOutputMU110::CommunicationOptions{};
   communicationOptions.PortPath(argv[1])
                       .BaudeRate(OWEN::DiscreteOutputMU110::CommunicationOptions::eBaudrate::_115200bps)
                       .Parity(OWEN::DiscreteOutputMU110::CommunicationOptions::eParity::NO)
                       .StopBits(false)
                       .DataBits(true)
                       .BaseAddr(17);
   auto discreteOutput = OWEN::DiscreteOutputMU110{communicationOptions};
//   auto const communicationOptions = discreteOutput.GetCommunicationOptions();
//   if (communicationOptions.has_value())
//   {
//     std::cout << communicationOptions.value() << std::endl;
//   }
//   else
//   {
//     std::cout << "error" << std::endl;
//   }

//   auto const discreteOutputOptions = discreteOutput.GetDiscreteOutputOptions();
//   if (discreteOutputOptions.has_value())
//   {
//     std::cout << discreteOutputOptions.value() << std::endl;
//   }
//   else
//   {
//     std::cout << "error" << std::endl;
//   }
   for (auto i = 0; i < 255; ++i)
   {
     auto const discreteOutputState = discreteOutput.GetDiscreteOutputsState();
     std::cout << "discreteoutputState: " << std::hex << static_cast<uint32_t>(discreteOutputState.value()) << std::endl;
     std::this_thread::sleep_for(std::chrono::seconds(1));
     discreteOutput.SetDiscreteOutputsState(i);
   }
   return 0;
}
