#include <OWEN/DiscreteOutputMU110.hpp>

#include "SerialPort.hpp"
#include "ModBus.hpp"

#include <iostream>

namespace OWEN {

namespace {

auto ToSerialPortType(DiscreteOutputMU110::CommunicationOptions::eParity parity) -> SerialPort::eParity
{
   using eParity = DiscreteOutputMU110::CommunicationOptions::eParity;
   switch(parity)
   {
      case eParity::EVEN:
         return SerialPort::eParity::even;
      case eParity::ODD:
         return SerialPort::eParity::odd;
      case eParity::NO:
      default:
         return SerialPort::eParity::none;
   }
}

static auto ToSerialPortType(DiscreteOutputMU110::CommunicationOptions::eBaudrate baudrate) -> uint32_t
{
   using eBaudrate = DiscreteOutputMU110::CommunicationOptions::eBaudrate;
   switch(baudrate)
   {
      case eBaudrate::_2400bps:
         return 2400;
      case eBaudrate::_4800bps:
         return 4800;
      case eBaudrate::_9600bps:
         return 9600;
      case eBaudrate::_14400bps:
         return 14400;
      case eBaudrate::_19200bps:
         return 19200;
      case eBaudrate::_28800bps:
         return 28800;
      case eBaudrate::_38400bps:
         return 38400;
      case eBaudrate::_57600bps:
         return 57600;
      case eBaudrate::_115200bps:
         return 115200;
      default: return 0;
   }
}

auto AutoFind(DiscreteOutputMU110::CommunicationOptions& communicationOptions, DiscreteOutputMU110::tFindProgress progress) -> SerialPort
{
   auto serialPortCreator = [](DiscreteOutputMU110::CommunicationOptions const& communicationOptions) -> SerialPort {
       return SerialPort(communicationOptions._portPath,
                         ToSerialPortType(communicationOptions._baudrate.value()),
                         ToSerialPortType(communicationOptions._parity.value()),
                         communicationOptions._stopBitsExtended.value() ? SerialPort::eStopBits::two : SerialPort::eStopBits::one,
                         communicationOptions._dataBitsExtended.value() ? static_cast<uint8_t>(8) : static_cast<uint8_t>(7));
   };
   using eBaudrate = DiscreteOutputMU110::CommunicationOptions::eBaudrate;
   using eParity = DiscreteOutputMU110::CommunicationOptions::eParity;


   auto const totalIterations = (static_cast<uint32_t>(eBaudrate::_115200bps) + 1) *
                                2 *
                                (static_cast<uint32_t>(eParity::EVEN) + 1) *
                                2 *
                                255;
   auto currentIt = 0;
   auto& co = communicationOptions;
   for (co._baseAddr = 1; co._baseAddr.value() < 255; ++co._baseAddr.value())
   {
      for (co._baudrate = eBaudrate::_2400bps; co._baudrate.value() <= eBaudrate::_115200bps;)
      {
         auto dataBitsExtended = 0;
         for (co._dataBitsExtended = false; dataBitsExtended < 2; ++dataBitsExtended)
         {
            for (co._parity = eParity::NO; co._parity.value() <= eParity::ODD;)
            {
               auto stopBitsExtended = 0;
               for (co._stopBitsExtended = false; stopBitsExtended < 2; ++stopBitsExtended)
               {
                  //for (co._lengthAddrExtended = false; co._lengthAddrExtended.value() < true; ++co._lengthAddrExtended.value())
                  {
                     ++currentIt;
                     std::cout << "Trying to get response from device with next serial configuration: \n" << co << std::endl;
                     try
                     {
                        auto serialPort = serialPortCreator(co);
                        std::cout << "Trying to send request" << std::endl;
                        auto modbus = ModBus(serialPort, co._baseAddr.value());
                        if (!modbus.ReadHoldingRegisters(0x0000, 1, 100).empty())
                        {
                           progress(totalIterations, totalIterations, co);
                           return serialPortCreator(co);
                        }
                        if (!progress(currentIt, totalIterations, co))
                        {
                           return serialPortCreator(co);
                        }
                     }
                     catch(boost::system::system_error ex)
                     {
                        std::cout << ex.what() << std::endl;
                     }
                  }
                  co._stopBitsExtended = stopBitsExtended == 0;
               }
               auto parity = static_cast<uint32_t>(co._parity.value());
               ++parity;
               co._parity = static_cast<eParity>(parity);
            }
            co._dataBitsExtended = dataBitsExtended == 0;
         }
         auto baudrate = static_cast<uint32_t>(co._baudrate.value());
         ++baudrate;
         co._baudrate = static_cast<eBaudrate>(baudrate);
      }
   }
}

} /// end namespace anonymous

class DiscreteOutputMU110::Impl
{
public:
  Impl(CommunicationOptions communicationOptions,
       bool neededToBeFound,
       tFindProgress progress)
    : _serialPort{/*!neededToBeFound
                  ? */SerialPort(communicationOptions._portPath,
                               ToSerialPortType(communicationOptions._baudrate.value()),
                               ToSerialPortType(communicationOptions._parity.value()),
                               communicationOptions._stopBitsExtended.value() ? SerialPort::eStopBits::two : SerialPort::eStopBits::one,
                               communicationOptions._dataBitsExtended.value() ? static_cast<uint8_t>(8) : static_cast<uint8_t>(7))
                  /*: AutoFind(communicationOptions, progress)*/}
    , _modBus{_serialPort, static_cast<uint8_t>(communicationOptions._baseAddr.value())}
  {
     if (_modBus.ReadHoldingRegisters(0x0000, 1, 100).empty())
     {
        throw std::runtime_error("Could not connected");
     }
  }

  auto GetCommunicationOptions() -> std::optional<CommunicationOptions>
  {
    auto const registers = _modBus.ReadHoldingRegisters(0x0000, 7);
    if (registers.size() < 7)
    {
      return {};
    }
    return CommunicationOptions{}.BaudeRate(static_cast<CommunicationOptions::eBaudrate>(registers[0]))
                                 .DataBits(registers[1])
                                 .Parity(static_cast<CommunicationOptions::eParity>(registers[2]))
                                 .StopBits(registers[3])
                                 .LengthAddr(registers[4])
                                 .BaseAddr(registers[5])
                                 .DelayAnswer(static_cast<uint8_t>(registers[6]));
  }

  bool SetCommunicationOptions(DiscreteOutputMU110::CommunicationOptions const& communicationOptions)
  {
    bool result{true};
    if (communicationOptions._baudrate.has_value())
    {
      result &= _modBus.WriteSingleHoldingRegister(0x0000, static_cast<uint16_t>(communicationOptions._baudrate.value()));
    }
    if (communicationOptions._dataBitsExtended.has_value())
    {
      result &= _modBus.WriteSingleHoldingRegister(0x0001, static_cast<uint16_t>(communicationOptions._dataBitsExtended.value()));
    }
    if (communicationOptions._parity.has_value())
    {
      result &= _modBus.WriteSingleHoldingRegister(0x0002, static_cast<uint16_t>(communicationOptions._parity.value()));
    }
    if (communicationOptions._stopBitsExtended.has_value())
    {
      result &= _modBus.WriteSingleHoldingRegister(0x0003, static_cast<uint16_t>(communicationOptions._stopBitsExtended.value()));
    }
    if (communicationOptions._lengthAddrExtended.has_value())
    {
      result &= _modBus.WriteSingleHoldingRegister(0x0004, static_cast<uint16_t>(communicationOptions._lengthAddrExtended.value()));
    }
    if (communicationOptions._baseAddr.has_value())
    {
      result &= _modBus.WriteSingleHoldingRegister(0x0005, static_cast<uint16_t>(communicationOptions._baseAddr.value()));
    }
    if (communicationOptions._delayAnswerMs.has_value())
    {
      result &= _modBus.WriteSingleHoldingRegister(0x0006, static_cast<uint16_t>(communicationOptions._delayAnswerMs.value()));
    }
    return result;
  }

#if 0
  auto GetDiscreteOutputOptions() -> std::optional<DiscreteOutputMU110::DiscreteOutputOptions>
  {
    auto registers = _modBus.ReadHoldingRegisters(0x0007, 13, 10000);
    auto registersContinue = _modBus.ReadHoldingRegisters(0x0007 + 13, 11, 10000);
    registers.insert(registers.end(), registersContinue.cbegin(), registersContinue.cend());
    if (registers.size() < 24)
    {
      return {};
    }
    return CounterOptions{}.DecPoint(static_cast<CounterOptions::eDecPoint>(registers[0]))
                           .InputMode(static_cast<CounterOptions::eInputMode>(registers[1]))
                           .OutputMode(static_cast<CounterOptions::eOutputMode>(registers[2]))
                           .SetPointMode(static_cast<CounterOptions::ePointMode>(registers[3]))
                           .ResetType(static_cast<CounterOptions::eResetType>(registers[4]))
                           .SetPoint1((((uint32_t)registers[5]) << 16) | (((uint32_t)registers[6]) & 0xFFFF))
                           .SetPoint2((((uint32_t)registers[7]) << 16) | (((uint32_t)registers[8]) & 0xFFFF))
                           .TimeOUT1((((uint32_t)registers[9]) << 16) | (((uint32_t)registers[10]) & 0xFFFF))
                           .TimeOUT2((((uint32_t)registers[11]) << 16) | (((uint32_t)registers[12]) & 0xFFFF))
                           .DecPointMult(static_cast<uint8_t>(registers[13]))
                           .Multiplexer((((uint32_t)registers[14]) << 16) | (((uint32_t)registers[15]) & 0xFFFF))
                           .MaxFreq(registers[16])
                           .MinControl((((uint32_t)registers[17]) << 16) | (((uint32_t)registers[18]) & 0xFFFF))
                           .LockKBD(static_cast<CounterOptions::eLockKBD>(registers[19]))
                           .ShowSetPoint(static_cast<CounterOptions::eShowSetPoint>(registers[20]))
                           .Brightness(registers[21])
                           .InputType(static_cast<CounterOptions::eInputType>(registers[22]))
                           .Password(registers[23]);
  }

  bool SetCounterOptions(DiscreteOutputMU110::DiscreteOutputOptions const& counterOptions)
  {
    bool result{true};
    if (counterOptions._decPoint.has_value())
    {
      result &= _modBus.WriteSingleHoldingRegister(0x0007, static_cast<uint16_t>(counterOptions._decPoint.value()));
    }
    if (counterOptions._inputMode.has_value())
    {
      result &= _modBus.WriteSingleHoldingRegister(0x0008, static_cast<uint16_t>(counterOptions._inputMode.value()));
    }
    if (counterOptions._outputMode.has_value())
    {
      result &= _modBus.WriteSingleHoldingRegister(0x0009, static_cast<uint16_t>(counterOptions._outputMode.value()));
    }
    if (counterOptions._pointMode.has_value())
    {
      result &= _modBus.WriteSingleHoldingRegister(0x000A, static_cast<uint16_t>(counterOptions._pointMode.value()));
    }
    if (counterOptions._resetType.has_value())
    {
      result &= _modBus.WriteSingleHoldingRegister(0x000B, static_cast<uint16_t>(counterOptions._resetType.value()));
    }
    if (counterOptions._point1Threshold.has_value())
    {
      result &= _modBus.WriteMultipleHoldingRegister(0x000C,
                                                   {static_cast<uint16_t>(counterOptions._point1Threshold.value() >> 16),
                                                    static_cast<uint16_t>(counterOptions._point1Threshold.value() & 0xFFFF)});
    }
    if (counterOptions._point2Threshold.has_value())
    {
      result &= _modBus.WriteMultipleHoldingRegister(0x000E,
                                                     {static_cast<uint16_t>(counterOptions._point2Threshold.value() >> 16),
                                                      static_cast<uint16_t>(counterOptions._point2Threshold.value() & 0xFFFF)});
    }
    if (counterOptions._timeout1.has_value())
    {
      result &= _modBus.WriteMultipleHoldingRegister(0x0010,
                                                     {static_cast<uint16_t>(counterOptions._timeout1.value() >> 16),
                                                      static_cast<uint16_t>(counterOptions._timeout1.value() & 0xFFFF)});
    }
    if (counterOptions._timeout2.has_value())
    {
      result &= _modBus.WriteMultipleHoldingRegister(0x0012,
                                                     {static_cast<uint16_t>(counterOptions._timeout2.value() >> 16),
                                                      static_cast<uint16_t>(counterOptions._timeout2.value() & 0xFFFF)});
    }
    if (counterOptions._decPointMult.has_value())
    {
      result &= _modBus.WriteSingleHoldingRegister(0x0014, static_cast<uint16_t>(counterOptions._decPointMult.value()));
    }
    if (counterOptions._multiplexer.has_value())
    {
      result &= _modBus.WriteMultipleHoldingRegister(0x0015,
                                                     {static_cast<uint16_t>(counterOptions._multiplexer.value() >> 16),
                                                      static_cast<uint16_t>(counterOptions._multiplexer.value() & 0xFFFF)});
    }
    if (counterOptions._maxFreq.has_value())
    {
      result &= _modBus.WriteSingleHoldingRegister(0x0017, static_cast<uint16_t>(counterOptions._maxFreq.value()));
    }
    if (counterOptions._minControl.has_value())
    {
      result &= _modBus.WriteMultipleHoldingRegister(0x0018,
                                                     {static_cast<uint16_t>(counterOptions._minControl.value() >> 16),
                                                      static_cast<uint16_t>(counterOptions._minControl.value() & 0xFFFF)});
    }
    if (counterOptions._lockKbd.has_value())
    {
      result &= _modBus.WriteSingleHoldingRegister(0x001A, static_cast<uint16_t>(counterOptions._lockKbd.value()));
    }
    if (counterOptions._showSetPoint.has_value())
    {
      result &= _modBus.WriteSingleHoldingRegister(0x001B, static_cast<uint16_t>(counterOptions._showSetPoint.value()));
    }
    if (counterOptions._brightness.has_value())
    {
      result &= _modBus.WriteSingleHoldingRegister(0x001C, static_cast<uint16_t>(counterOptions._brightness.value()));
    }
    if (counterOptions._inputType.has_value())
    {
      result &= _modBus.WriteSingleHoldingRegister(0x001D, static_cast<uint16_t>(counterOptions._inputType.value()));
    }
    if (counterOptions._password.has_value())
    {
      result &= _modBus.WriteSingleHoldingRegister(0x001E, static_cast<uint16_t>(counterOptions._password.value()));
    }
    return result;
  }
#endif
  auto GetDiscreteOutputsState() -> std::optional<uint8_t>
  {
    auto const registers = _modBus.ReadHoldingRegisters(0x0032, 1);
    return (registers.size() != 1)
              ? std::optional<uint8_t>{}
              : std::optional<uint8_t>{registers[0] & 0x00FF};
  }

  bool SetDiscreteOutputsState(uint8_t state)
  {
    return _modBus.WriteMultipleHoldingRegister(0x0032, std::vector<uint16_t>{state}, 1000);
  }

private:
  SerialPort _serialPort;
  ModBus _modBus;
};

DiscreteOutputMU110::DiscreteOutputMU110(CommunicationOptions const& communicationOptions, bool neededToBeFound, tFindProgress progress)
  : pImpl{std::make_unique<Impl>(communicationOptions, neededToBeFound, progress)}
{
}

DiscreteOutputMU110::~DiscreteOutputMU110() = default;

DiscreteOutputMU110::DiscreteOutputMU110(DiscreteOutputMU110&&) noexcept = default;

DiscreteOutputMU110& DiscreteOutputMU110::operator=(DiscreteOutputMU110&&) noexcept = default;

bool DiscreteOutputMU110::SetCommunicationOptions(CommunicationOptions const& communicationOptions)
{
   return pImpl->SetCommunicationOptions(communicationOptions);
}

auto DiscreteOutputMU110::GetCommunicationOptions() -> std::optional<CommunicationOptions>
{
   return pImpl->GetCommunicationOptions();
}

//bool DiscreteOutputMU110::SetDiscreteOutputOptions(DiscreteOutputOptions const& counterOptions)
//{
//   return pImpl->SetDiscreteOutputOptions(counterOptions);
//}
//
//auto DiscreteOutputMU110::GetDiscreteOutputOptions() -> std::optional<DiscreteOutputOptions>
//{
//   return pImpl->GetDiscreteOutputOptions();
//}

auto DiscreteOutputMU110::GetDiscreteOutputsState() -> std::optional<uint8_t>
{
  return pImpl->GetDiscreteOutputsState();
}

bool DiscreteOutputMU110::SetDiscreteOutputsState(uint8_t state)
{
  return pImpl->SetDiscreteOutputsState(state);
}

auto operator<<(std::ostream& out, OWEN::DiscreteOutputMU110::CommunicationOptions::eBaudrate const& baudrate) -> std::ostream&
{
  using eBaudrate = OWEN::DiscreteOutputMU110::CommunicationOptions::eBaudrate;
  switch(baudrate)
  {
    case eBaudrate::_2400bps:
      out << "2400bps";
      break;
    case eBaudrate::_4800bps:
      out << "4800bps";
      break;
    case eBaudrate::_9600bps:
      out << "9600bps";
      break;
    case eBaudrate::_14400bps:
      out << "14400bps";
      break;
    case eBaudrate::_19200bps:
      out << "19200bps";
      break;
    case eBaudrate::_28800bps:
      out << "28800bps";
      break;
    case eBaudrate::_38400bps:
      out << "38400bps";
      break;
    case eBaudrate::_57600bps:
      out << "57600bps";
      break;
    case eBaudrate::_115200bps:
      out << "115200bps";
      break;
    default:
      out << "-";
  }
  return out;
}

auto operator<<(std::ostream& out, OWEN::DiscreteOutputMU110::CommunicationOptions::eParity const& parity) -> std::ostream&
{
  using eParity = OWEN::DiscreteOutputMU110::CommunicationOptions::eParity;
  switch(parity)
  {
    case eParity::NO:
      out << "NO";
      break;
    case eParity::ODD:
      out << "ODD";
      break;
    case eParity::EVEN:
      out << "EVEN";
      break;
    default:
      out << "-";
  }
  return out;
}

auto operator<<(std::ostream& out, OWEN::DiscreteOutputMU110::CommunicationOptions const& communicationOptions) -> std::ostream&
{
   using eBaudrate = OWEN::DiscreteOutputMU110::CommunicationOptions::eBaudrate;
   using eParity = OWEN::DiscreteOutputMU110::CommunicationOptions::eParity;
   auto const baudrate = communicationOptions._baudrate.has_value()
                            ? communicationOptions._baudrate.value() : static_cast<eBaudrate>(-1);
   auto const databits = communicationOptions._dataBitsExtended.has_value()
                            ? (communicationOptions._dataBitsExtended.value() ? '8' : '7') : '-';
   auto const parity = communicationOptions._parity.has_value()
                          ? communicationOptions._parity.value() : static_cast<eParity>(-1);
   auto const stopBits = communicationOptions._stopBitsExtended.has_value()
                            ? (communicationOptions._stopBitsExtended.value() ? '2' : '1') : '-';
   auto const lengthAddr = communicationOptions._lengthAddrExtended.has_value()
                              ? (communicationOptions._lengthAddrExtended.value() ? "11bit - [0..2047]" : "8bit - [1..255]") : "-";
   auto const baseAddr = communicationOptions._baseAddr.has_value()
                            ? std::to_string(communicationOptions._baseAddr.value()) : "-";
   auto const delayAnswerMs = communicationOptions._delayAnswerMs.has_value()
                            ? std::to_string(communicationOptions._delayAnswerMs.value()) : "-";
   out << "baudrate: " << baudrate << '\n'
       << "dataBits: " << databits << '\n'
       << "parity: " << parity << '\n'
       << "stopBits: " << stopBits << '\n'
       << "lengthAddr: " << lengthAddr << '\n'
       << "baseAddr: " << baseAddr << '\n'
       << "delayAnswerMs: " << delayAnswerMs;
   return out;
}

} /// end namespace OWEN
