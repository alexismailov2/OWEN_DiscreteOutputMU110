#pragma once

#include <memory>
#include <string>
#include <optional>

namespace OWEN {

class DiscreteOutputMU110
{
public:
  class CommunicationOptions
  {
  public:
    enum class eBaudrate {
      _2400bps,
      _4800bps,
      _9600bps,
      _14400bps,
      _19200bps,
      _28800bps,
      _38400bps,
      _57600bps,
      _115200bps
    };

    enum class eParity {
      NO,
      EVEN,
      ODD
    };

  public:
    /**
     * Set baudrate.
     * @param baudrate all values from eBaudrate enumeration.
     * @return reference to CommunicationOptions.
     */
    CommunicationOptions& BaudeRate(eBaudrate baudrate) {
      _baudrate = baudrate;
      return *this;
    }

    /**
     * Set usual or extended data bits.
     * @param dataBitsExtended true - 8bit, false - 7bit.
     * @return reference to CommunicationOptions.
     */
    CommunicationOptions& DataBits(bool dataBitsExtended) {
      _dataBitsExtended = dataBitsExtended;
      return *this;
    }

    /**
     * Set parity checking.
     * @param parity NO - no parity checking, EVEN - even parity checking, ODD - odd parity checking.
     * @return reference to CommunicationOptions.
     */
    CommunicationOptions& Parity(eParity parity) {
       _parity = parity;
      return *this;
    }

    /**
     * Set usual or extended stop bits count.
     * @param stopBitsExtended true - 2 stop bit, false 1 stop bit.
     * @return reference to CommunicationOptions.
     */
    CommunicationOptions& StopBits(bool stopBitsExtended) {
      _stopBitsExtended = stopBitsExtended;
      return *this;
    }

    /**
     * Set usial or extended length of addr.
     * @param extended if true - 11 bit, false - 8 bit.
     * @return reference to CommunicationOptions.
     */
    CommunicationOptions& LengthAddr(bool lengthAddrExtended) {
      _lengthAddrExtended = lengthAddrExtended;
      return *this;
    }

    /**
     * Set base addr.
     * @param baseAddr base addr.
     * @return reference to CommunicationOptions.
     */
    CommunicationOptions& BaseAddr(uint16_t baseAddr) {
      if (baseAddr < 1 || baseAddr > 2047)
      {
        throw std::runtime_error("BaseAddr should be in range 1 - 2047.");
      }
      _baseAddr = baseAddr;
      return *this;
    }

    /**
     * Set delay answer for RS485 interface.
     * @param delayMs delay for answer in milliseconds for RS485 only.
     * @return reference to CommunicationOptions.
     */
    CommunicationOptions& DelayAnswer(uint8_t delayAnswerMs) {
      if (delayAnswerMs > 45)
      {
        throw std::runtime_error("DelayAnswer should be in range 0 - 45.");
      }
      _delayAnswerMs = delayAnswerMs;
      return *this;
    }

    std::optional<eBaudrate> _baudrate{};
    std::optional<bool> _dataBitsExtended{};
    std::optional<eParity> _parity{};
    std::optional<bool> _stopBitsExtended{};
    std::optional<bool> _lengthAddrExtended{};
    std::optional<uint16_t> _baseAddr{};
    std::optional<uint8_t> _delayAnswerMs{};
  };

//  class DiscreteOutputOptions {
//  public:
//    DiscreteOutputOptions& DecPoint(eDecPoint decPoint)
//    {
//      _decPoint = decPoint;
//      return *this;
//    }
//
//    CounterOptions& InputMode(eInputMode inputMode)
//    {
//      _inputMode = inputMode;
//      return *this;
//    }
//
//    CounterOptions& OutputMode(eOutputMode outputMode)
//    {
//      _outputMode = outputMode;
//      return *this;
//    }
//
//    /// ...
//    std::optional<std::vector<int32>> _decPoint{};
//    std::optional<eInputMode> _inputMode{};
//    std::optional<eOutputMode> _outputMode{};
//  };

public:
  DiscreteOutputMU110(std::string const& portPath = "/dev/cu.usbserial-14610", uint32_t baudrate = 115200, uint8_t deviceAddress = 0x10);
  ~DiscreteOutputMU110();
  DiscreteOutputMU110(DiscreteOutputMU110&&) noexcept;
  DiscreteOutputMU110& operator=(DiscreteOutputMU110&&) noexcept;

  bool SetCommunicationOptions(CommunicationOptions const& communicationOptions);

  auto GetCommunicationOptions() -> std::optional<CommunicationOptions>;

//  bool SetDiscreteOutputOptions(DiscreteOutputOptions const& discreteOutputOptions);
//
//  auto GetDiscreteOutputOptions() -> std::optional<DiscreteOutputOptions>;

  auto GetDiscreteOutputsState() -> std::optional<uint8_t>;

  bool SetDiscreteOutputsState(uint8_t state);

private:
  class Impl;
  std::unique_ptr<Impl> pImpl;
};

std::ostream& operator<<(std::ostream& out, OWEN::DiscreteOutputMU110::CommunicationOptions const& communicationOptions);

#if 0
std::ostream& operator<<(std::ostream& out, OWEN::DiscreteOutputMU110::DiscreteOutputOptions const& counterOptions);

#endif
} /// end namespace OWEN