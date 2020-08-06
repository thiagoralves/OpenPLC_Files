/*
ModbusSerial.cpp - Source for Modbus Serial Library
Copyright (C) 2014 André Sarmento Barbosa
*/
#include "ModbusSerial.h"

ModbusSerial::ModbusSerial() {

}

bool ModbusSerial::setSlaveId(byte slaveId){
  _slaveId = slaveId;
  return true;
}

byte ModbusSerial::getSlaveId() {
  return _slaveId;
}

#ifndef __AVR_ATmega32U4__
#ifndef DEBUG_MODE
bool ModbusSerial::config(HardwareSerial* port, long baud, int txPin) {
  this->_port = port;
  this->_txPin = txPin;
  (*port).begin(baud);

  if (txPin >= 0) {
    pinMode(txPin, OUTPUT);
    digitalWrite(txPin, LOW);
  }

  // Modbus states that a baud rate higher than 19200 must use a fixed 750 us
  // for inter character time out and 1.75 ms for a frame delay for baud rates
  // below 19200 the timing is more critical and has to be calculated.
  // E.g. 9600 baud in a 11 bit packet is 9600/11 = 872 characters per second
  // In milliseconds this will be 872 characters per 1000ms. So for 1 character
  // 1000ms/872 characters is 1.14583ms per character and finally modbus states
  // an inter-character must be 1.5T or 1.5 times longer than a character. Thus
  // 1.5T = 1.14583ms * 1.5 = 1.71875ms. A frame delay is 3.5T.
  // Thus the formula is T1.5(us) = (1000ms * 1000(us) * 1.5 * 11bits)/baud
  // 1000ms * 1000(us) * 1.5 * 11bits = 16500000 can be calculated as a constant

  if (baud > 19200)
  _t15 = 750;
  else
  _t15 = 16500000/baud; // 1T * 1.5 = T1.5

  /* The modbus definition of a frame delay is a waiting period of 3.5 character times
  between packets. This is not quite the same as the frameDelay implemented in
  this library but does benifit from it.
  The frameDelay variable is mainly used to ensure that the last character is
  transmitted without truncation. A value of 2 character times is chosen which
  should suffice without holding the bus line high for too long.*/

  _t35 = _t15 * 3.5;

  return true;
}
#endif

#ifdef DEBUG_MODE
bool ModbusSerial::config(HardwareSerial* port,
  HardwareSerial* DebugSerialPort, long baud, int txPin) {
    this->_port = port;
    this->_txPin = txPin;
    (*port).begin(baud);

    DebugPort = DebugSerialPort;

    if (txPin >= 0) {
      pinMode(txPin, OUTPUT);
      digitalWrite(txPin, LOW);
    }

    if (baud > 19200)
    _t15 = 750;
    else
    _t15 = 16500000/baud; // 1T * 1.5 = T1.5

    _t35 = _t15 * 3.5;

    return true;
  }
  #endif

  #ifdef USE_SOFTWARE_SERIAL
  bool ModbusSerial::config(SoftwareSerial* port, long baud, int txPin) {
    this->_port = port;
    this->_txPin = txPin;
    (*port).begin(baud);

    delay(2000);

    if (txPin >= 0) {
      pinMode(txPin, OUTPUT);
      digitalWrite(txPin, LOW);
    }

    if (baud > 19200)
    _t15 = 750;
    else
    _t15 = 16500000/baud; // 1T * 1.5 = T1.5

    _t35 = _t15 * 3.5;

    return true;
  }
  #endif
  #endif

  #ifdef __AVR_ATmega32U4__
  #ifdef DEBUG_MODE
  bool ModbusSerial::config(HardwareSerial* port,
    Serial_* DebugSerialPort, long baud, int txPin) {
      this->_port = port;
      this->_txPin = txPin;
      (*port).begin(baud);

      DebugPort = DebugSerialPort;

      if (txPin >= 0) {
        pinMode(txPin, OUTPUT);
        digitalWrite(txPin, LOW);
      }

      if (baud > 19200)
      _t15 = 750;
      else
      _t15 = 16500000/baud; // 1T * 1.5 = T1.5

      _t35 = _t15 * 3.5;

      return true;
  }
  #else
  bool ModbusSerial::config(Serial_* port, long baud, int txPin) {
    this->_port = port;
    this->_txPin = txPin;
    (*port).begin(baud);
    while (!(*port));

    if (txPin >= 0) {
      pinMode(txPin, OUTPUT);
      digitalWrite(txPin, LOW);
    }

    if (baud > 19200)
    _t15 = 750;
    else
    _t15 = 16500000/baud; // 1T * 1.5 = T1.5

    _t35 = _t15 * 3.5;

    return true;
  }
  #endif
  #endif

  bool ModbusSerial::receive(byte* frame) {
    //first byte of frame = address
    byte address = frame[0];
    //Last two bytes = crc
    u_int crc = ((frame[_len - 2] << 8) | frame[_len - 1]);

    //Slave Check
    if (address != 0xFF && address != this->getSlaveId()) {
      return false;
    }

    //CRC Check
    if (crc != this->calcCrc(_frame[0], _frame+1, _len-3)) {
      return false;
    }

    //PDU starts after first byte
    //framesize PDU = framesize - address(1) - crc(2)
    this->receivePDU(frame+1);
    //No reply to Broadcasts
    if (address == 0xFF) _reply = MB_REPLY_OFF;
    return true;
  }

  bool ModbusSerial::send(byte* frame) {
    byte i;

    if (this->_txPin >= 0) {
      digitalWrite(this->_txPin, HIGH);
      delay(1);
    }

    for (i = 0 ; i < _len ; i++) {
      (*_port).write(frame[i]);
    }

    (*_port).flush();
    delayMicroseconds(_t35);

    if (this->_txPin >= 0) {
      digitalWrite(this->_txPin, LOW);
    }
  }

  bool ModbusSerial::sendPDU(byte* pduframe) {
    if (this->_txPin >= 0) {
      digitalWrite(this->_txPin, HIGH);
      delay(1);
    }

    delayMicroseconds(_t35);

    //Send slaveId
    (*_port).write(_slaveId);

    //Send PDU
    byte i;
    for (i = 0 ; i < _len ; i++) {
      (*_port).write(pduframe[i]);
    }

    //Send CRC
    word crc = calcCrc(_slaveId, _frame, _len);
    (*_port).write(crc >> 8);
    (*_port).write(crc & 0xFF);

    (*_port).flush();
    delayMicroseconds(_t35);

    if (this->_txPin >= 0) {
      digitalWrite(this->_txPin, LOW);
    }

    #ifdef DEBUG_MODE
    (*DebugPort).println("SENT Serial RESPONSE");
    (*DebugPort).print(_slaveId);
    (*DebugPort).print(':');
    for (int i = 0 ; i < _len ; i++) {
      (*DebugPort).print(_frame[i]);
      (*DebugPort).print(':');
    }
    (*DebugPort).print(crc >> 8);
    (*DebugPort).print(':');
    (*DebugPort).print(crc & 0xFF);
    (*DebugPort).print(':');
    (*DebugPort).println();
    (*DebugPort).println(F("-----------------"));
    #endif
  }

  void ModbusSerial::task() {
    _len = 0;

    while ((*_port).available() > _len)	{
      _len = (*_port).available();
      delayMicroseconds(_t15);
    }

    if (_len == 0) return;

    byte i;
    _frame = (byte*) malloc(_len);
    for (i=0 ; i < _len ; i++) _frame[i] = (*_port).read();

    #ifdef DEBUG_MODE
    (*DebugPort).println(F("GOT Serial CMD"));
    for (int i=0 ; i < _len ; i++) {
      (*DebugPort).print(_frame[i], DEC);
      (*DebugPort).print(":");
      //(*DebugPort).println(sendbuffer[i], HEX);
    }
    (*DebugPort).println();
    (*DebugPort).println(F("-----------------"));
    #endif

    if (this->receive(_frame)) {
      if (_reply == MB_REPLY_NORMAL)
      this->sendPDU(_frame);
      else
      if (_reply == MB_REPLY_ECHO)
      this->send(_frame);
    }

    free(_frame);
    _len = 0;
  }

  word ModbusSerial::calcCrc(byte address, byte* pduFrame, byte pduLen) {
    byte CRCHi = 0xFF, CRCLo = 0x0FF, Index;

    Index = CRCHi ^ address;
    CRCHi = CRCLo ^ _auchCRCHi[Index];
    CRCLo = _auchCRCLo[Index];

    while (pduLen--) {
      Index = CRCHi ^ *pduFrame++;
      CRCHi = CRCLo ^ _auchCRCHi[Index];
      CRCLo = _auchCRCLo[Index];
    }

    return (CRCHi << 8) | CRCLo;
  }
