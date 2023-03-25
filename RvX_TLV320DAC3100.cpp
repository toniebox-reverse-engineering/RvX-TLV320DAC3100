#include "RvX_TLV320DAC3100.h"

void RvX_TLV320DAC3100::delayTask(uint16_t ms) {
  delay(ms);
}

bool RvX_TLV320DAC3100::begin(uint8_t i2caddr) {
    _wire = &Wire;
    _wire->begin();
    _i2caddr = i2caddr;

    if (!_initDACI2C())
        return false;


    if (!setVolume(VOL_MIN))
        return false;
    send(ADDR::PAGE_CONTROL, PAGE::SERIAL_IO);
    send(ADDR_P0_SERIAL::DAC_VOL_CTRL, 0x00);

    /*
    uint8_t headsetDetect = readByte(ADDR_P0_SERIAL::HEADSET_DETECT);
    Log.info("Headset detect=%B", headsetDetect); // Always no Headset detected?! TODO
    if ((headsetDetect & 0b00100000) == 0b00100000) {
        Events.handleHeadphoneEvent(HeadphoneEvent::INSERTED);
    } else { 
        Events.handleHeadphoneEvent(HeadphoneEvent::REMOVED);
    };
    */
   return true;
}


bool RvX_TLV320DAC3100::send(uint8_t target_register, uint8_t data) {
    return send(_i2caddr, target_register, data);
}
bool RvX_TLV320DAC3100::send(ADDR target_register, PAGE data) {
    return send((uint8_t)target_register, (uint8_t)data);
}
bool RvX_TLV320DAC3100::send(ADDR_P0_SERIAL target_register, uint8_t data) {
    return send((uint8_t)target_register, data);
}
bool RvX_TLV320DAC3100::send(ADDR_P1_DAC_OUT target_register, uint8_t data) {
    return send((uint8_t)target_register, data);
}
bool RvX_TLV320DAC3100::send(ADDR_P3_MCLK target_register, uint8_t data) {
    return send((uint8_t)target_register, data);
}

uint8_t RvX_TLV320DAC3100::readByte(uint8_t source_register) {
    return readByte(_i2caddr, source_register);
}
uint8_t RvX_TLV320DAC3100::readByte(ADDR source_register) {
    return readByte((uint8_t)source_register);
}
uint8_t RvX_TLV320DAC3100::readByte(ADDR_P0_SERIAL source_register) {
    return readByte((uint8_t)source_register);
}
uint8_t RvX_TLV320DAC3100::readByte(ADDR_P1_DAC_OUT source_register) {
    return readByte((uint8_t)source_register);
}
uint8_t RvX_TLV320DAC3100::readByte(ADDR_P3_MCLK source_register) {
    return readByte((uint8_t)source_register);
}

bool RvX_TLV320DAC3100::increaseVolume() {
    bool result = false;
    uint16_t (*pBeep)[2] = ofwButtonFreqTable[getSampleRateIndex()];

    if (current_volume < VOL_MAX) {
        setVolume(current_volume+VOL_STEP);
        beepRaw(pBeep[0][0], pBeep[0][1], 0x000140); //beepRaw(0x278A, 0x79BD, 0x000140); //16kHz - 799,69hz/799,80hz/16
        //beepMidi(79,50,true);
        result = true;
    } else {
        beepRaw(pBeep[1][0], pBeep[1][1], 0x000140); //beepRaw(0x30F9, 0x763F, 0x000140); //16kHz - 999,77hz/1000,54hz/20
        delayTask(50);
        beepRaw(pBeep[1][0], pBeep[1][1], 0x000140);
        //beepMidi(83,50,true);
    }
    return result;
}
bool RvX_TLV320DAC3100::decreaseVolume() {
    bool result = false;
    uint16_t (*pBeep)[2] = ofwButtonFreqTable[getSampleRateIndex()];
    
    if (current_volume > VOL_MIN) {
        setVolume(current_volume-VOL_STEP);
        beepRaw(pBeep[2][0], pBeep[2][1], 0x000140); //beepRaw(0x18F5, 0x7D87, 0x000140); //16kHz - 499,71hz/501,34hz/10
        //beepMidi(71, 50, true);
        result = true;
    } else {
        beepRaw(pBeep[3][0], pBeep[3][1], 0x000140); //beepRaw(0x0F0A, 0x7F1A, 0x000140); //16kHz - 299,89hz/301,57hz/6
        delayTask(50);
        beepRaw(pBeep[3][0], pBeep[3][1], 0x000140);
        //beepMidi(62, 50, true);
    }
    return result;
}
uint8_t RvX_TLV320DAC3100::convertDacVol2BeepVol(uint8_t dacVol) {
    //DAC  Range -63.5dB - +24.0dB
    //BEEP Range -61.0dB - +02.0dB
    int8_t dbVal = (dacVol-64*2)/2;

    if (dbVal>2) {
        dbVal = 2;
    } else if (dbVal<-61) {
        dbVal = -61;
    }

    int8_t beepVol = 2-dbVal;

    return beepVol & 0x3F;
}

bool RvX_TLV320DAC3100::setVolume(uint8_t volume) {
    int8_t volumeConv = (int8_t)(volume-0x7F);
    send(ADDR::PAGE_CONTROL, PAGE::SERIAL_IO);
    send(ADDR_P0_SERIAL::DAC_VOL_L_CTRL, volumeConv);
    send(ADDR_P0_SERIAL::DAC_VOL_R_CTRL, volumeConv);
    uint8_t flag_reg = 0;
    for(uint8_t i=0; i<50; i++) {
        flag_reg = readByte(ADDR_P0_SERIAL::DAC_FLAG_REG) & 0b00010001;
        if (flag_reg == 0b00010001) {
    current_volume = volume;
            return true;
        }
        delayTask(1);
    }
    return false;
}

bool RvX_TLV320DAC3100::send_raw(uint8_t data) {
    if (!_wire->write(data)) {
        return false;
    }
    return true;
}
bool RvX_TLV320DAC3100::send(uint8_t address, uint8_t target_register, uint8_t data) {
    _wire->beginTransmission(address);
    if (!send_raw(target_register)) return false;
    if (!send_raw(data)) return false;
    
    uint8_t result = _wire->endTransmission(false);
    if (!result) return true;
    return false;
}

uint8_t RvX_TLV320DAC3100::readByte(uint8_t address, uint8_t source_register) {
    _wire->beginTransmission(address);
    if (!send_raw(source_register)) return false;
    _wire->endTransmission(false);
    if (!_wire->requestFrom(address ,1)) return false;
    int result = _wire->read();
    if (result == -1) return false;
    return (uint8_t)result;
}

bool RvX_TLV320DAC3100::_initDACI2C() {
    //Extracted from logic analyzer capture of box
    if (!send(ADDR::PAGE_CONTROL, PAGE::SERIAL_IO))
        return false;
    send(ADDR_P0_SERIAL::SOFTWARE_RESET, 0x01);     //Self-clearing software reset for control register

    send(ADDR_P0_SERIAL::CLOCKGEN_MUX, 0x07);       //0000:reserved, 01:PLL_CLKIN=BCLK, 11:CODEC_CLKIN=PLL_CLK 
    send(ADDR_P0_SERIAL::PLL_J_VAL, 0x20);          //00:reserved, 100000:PLL multiplier J=32 (0x20)
    send(ADDR_P0_SERIAL::PLL_D_VAL_MSB, 0x00);      //00:reserved, 000000:fraktional multiplier D-value = 0
    send(ADDR_P0_SERIAL::PLL_D_VAL_LSB, 0x00);      //00:reserved, 000000:fraktional multiplier D-value = 0
    send(ADDR_P0_SERIAL::PLL_P_R_VAL, 0x96);        //1:PLL is power up, 001:PLL divider P=1, 110:PLL multiplier R=6
    send(ADDR_P0_SERIAL::DAC_NDAC_VAL, 0x84);       //1:NDAC divider powered up, 0000100:DAC NDAC divider=4
    send(ADDR_P0_SERIAL::DAC_MDAC_VAL, 0x86);       //1:MDAC divider powered up, 0000100:DAC MDAC divider=6
    send(ADDR_P0_SERIAL::DAC_DOSR_VAL_MSB, 0x01);   //000000:reserved, 01:DAC OSR MSB =256
    send(ADDR_P0_SERIAL::DAC_DOSR_VAL_LSB, 0x00);   //00000000:DAC OSR LSB

    delayTask(10);            //w PLL Start-Up

    send(ADDR_P0_SERIAL::CODEC_IF_CTRL1, 0x00);     //00:Codec IF=I2S, 00: Codec IF WL=16 bits, 0:BCLK=Input, 0:WCKL=Output, 0:reserved        // w IF statt INT
    send(ADDR_P0_SERIAL::DAC_PROC_BLOCK_SEL, 0x19); //000:reserved, 11001:DAC signal-processing block PRB_P25

    send(ADDR::PAGE_CONTROL, PAGE::DAC_OUT_VOL);
    send(ADDR_P1_DAC_OUT::HP_OUT_POP_REM_SET, 0x4E);          //0:simultan.DAC/HP/SP, 1001:power-on-time=1.22s*,11:drv.ramp-up=3.9ms,0:CM voltage 
    send(ADDR_P1_DAC_OUT::OUT_PGA_RAMP_DOWN_PER_CTRL, 0x70);  //0:reserved, 111=30.5ms*, 0000:reserved *8.2MHz
    send(ADDR_P1_DAC_OUT::DAC_LR_OUT_MIX_ROUTING, 0x44);      //01:DAC_L to MixAmp_L,00:AIN1/2 not routed, 01:DAC_R to MIxAmp_R, 00:AIN1/2 not routed
    send(ADDR_P1_DAC_OUT::MICBIAS, 0x0B);                     //0:SwPowDwn not enabled, 000:reserved, 1:MICBIAS powered up, 0:reserved, 11:MICBIAS=AVDD
    send(ADDR_P1_DAC_OUT::HP_DRIVER_CTRL, 0xE0);              //000:Debounce Time=0us, 01:DAC perform.increased, 1:HPL output=lineout, 1:HPR output=lineout, 0:reserved ??? LINE

    send(ADDR::PAGE_CONTROL, PAGE::MCLK_DIVIDER);
    send(ADDR_P3_MCLK::TIMER_CLK_MCLK_DIV, 0x01);             //0:Internal oscillator for delay timer, 0000001: MCLK divider=1 

    send(ADDR::PAGE_CONTROL, PAGE::SERIAL_IO);
    send(ADDR_P0_SERIAL::HEADSET_DETECT, 0x8C);               //1:Headset detection enabled, RR, 011:Debounce Prog.Glitch=128ms, 00:Debounce Prog.Glitch=0ms
    send(ADDR_P0_SERIAL::INT1_CTRL_REG, 0x80);                //1:Headset-insertion detect IRQ INT1, 0:Button-press detect, ...., 0=INT1 is only one pulse 2ms
    send(ADDR_P0_SERIAL::GPIO1_INOUT_CTRL, 0x14);             //XX:reserved, 0101:GPIO1=INT1 output, X=GPIO1 input buffer value, GPIO1 Output=X

    //send(0x2E); Excel 161

    // PAUSE 0,2s
    //read 0x18 addr



    send(ADDR::PAGE_CONTROL, PAGE::DAC_OUT_VOL);  // MUTE ALL
    send(ADDR_P1_DAC_OUT::L_VOL_TO_HPL, 0x7F);    // HPL Vol -oo
    send(ADDR_P1_DAC_OUT::R_VOL_TO_HPR, 0x7F);    // HPL Vol -oo
    send(ADDR_P1_DAC_OUT::L_VOL_TO_SPK, 0x7F);    // SPK Vol -oo


    // PAUSE 50ms
    delayTask(50);            //w Ramp

                                                // MUTE HP Driver AND SPK Driver
    send(ADDR_P1_DAC_OUT::HPL_DRIVER, 0x02);    // HPL driver is muted ??? must 1
    send(ADDR_P1_DAC_OUT::HPR_DRIVER, 0x02);    // HPR driver is muted ??? must 1
    send(ADDR_P1_DAC_OUT::SPK_DRIVER, 0x00);    // SPK driver is muted

    send(ADDR_P1_DAC_OUT::SPK_DRIVER, 0x04);    // an TEST   gehört hier nicht hin
 
    //PAUSE 50ms
    delayTask(50);            //w Ramp

                                                // AMPS Power Down
    send(ADDR_P1_DAC_OUT::HP_DRIVERS, 0x00);    // HPL HPR Driver Power Down  ??? must 1
    //send(ADDR_P1_DAC_OUT::SPK_AMP, 0x00);       // falscher Wert SPK Amp Power Down ??? must 11
    send(ADDR_P1_DAC_OUT::SPK_AMP, 0x06);       // SPK Amp Power Down ??? must 000011


    //PAUSE 50ms
    delayTask(50);            //w Ramp

    send(ADDR_P1_DAC_OUT::HPL_DRIVER, 0x06);  // HPL driver 0dB, not muted 
    send(ADDR_P1_DAC_OUT::HPR_DRIVER, 0x06); // HPR drvier 0dB, not muted
    //send(ADDR_P1_DAC_OUT::HP_DRIVERS, 0xC2); // Falscher Wert must 1
    send(ADDR_P1_DAC_OUT::HP_DRIVERS, 0xC4); // HPL HPR is power up, 1,35V, Shortcut=Error ??? must 1
    send(ADDR_P1_DAC_OUT::L_VOL_TO_HPL, 0x92);  // ??? Aux to HP ???
    send(ADDR_P1_DAC_OUT::R_VOL_TO_HPR, 0x92);  // ??? Aux to HP ???

    send(ADDR_P1_DAC_OUT::L_VOL_TO_SPK, 0x00);  // !!! FEHLTE !!!
    send(ADDR_P1_DAC_OUT::SPK_AMP, 0x86);       // !!! FEHLTE !!! SPK Amp Power Up



    // PAUSE 50ms
    delayTask(50);            //w Ramp


    send(ADDR::PAGE_CONTROL, PAGE::SERIAL_IO);
    send(ADDR_P0_SERIAL::DAC_DATA_PATH_SETUP, 0xD5);  // DAC power on, Left=left, Right=Right, DAC Softstep HP STEREO
    //send(ADDR_P0_SERIAL::DAC_DATA_PATH_SETUP, 0xF1);  // DAC power on, Left=left, Right=Right, DAC Softstep SPEAKER MONO
    send(ADDR_P0_SERIAL::DAC_VOL_L_CTRL, 0xDC);
    send(ADDR_P0_SERIAL::DAC_VOL_R_CTRL, 0xDC);
    //Excel 219
    // Extract END
    send(ADDR::PAGE_CONTROL, PAGE::DAC_OUT_VOL);
    send(ADDR_P1_DAC_OUT::L_VOL_TO_SPK, 128);
    return true;
}

void RvX_TLV320DAC3100::beepRaw(uint16_t sin, uint16_t cos, uint32_t length) {
    beepRaw(sin, cos, length, convertDacVol2BeepVol(current_volume));
}
void RvX_TLV320DAC3100::beepRaw(uint16_t sin, uint16_t cos, uint32_t length, uint8_t volume) {
    send(ADDR::PAGE_CONTROL, PAGE::SERIAL_IO);

    send(ADDR_P0_SERIAL::DAC_VOL_CTRL, 0x0C); //mute DACs //optional
    //f 30 26 xxx1xxx1 # wait for DAC gain flag to be set
    while ((readByte(ADDR_P0_SERIAL::DAC_FLAG_REG) & 0b00010001) != 0b00010001) { delayTask(1); }
    //send(ADDR_P0_SERIAL::DAC_NDAC_VAL, 0x02); //power down NDAC divider - Page 41 (but makes glitches?!)

    send(ADDR_P0_SERIAL::BEEP_LEN_MSB, (length>>16)&0xFF);
    send(ADDR_P0_SERIAL::BEEP_LEN_MID, (length>>8)&0xFF);
    send(ADDR_P0_SERIAL::BEEP_LEN_LSB, length);

    send(ADDR_P0_SERIAL::BEEP_SIN_MSB, (sin>>8)&0xFF);
    send(ADDR_P0_SERIAL::BEEP_SIN_LSB, sin);

    send(ADDR_P0_SERIAL::BEEP_COS_MSB, (cos>>8)&0xFF);
    send(ADDR_P0_SERIAL::BEEP_COS_LSB, cos);

    send(ADDR_P0_SERIAL::BEEP_R_GEN, 0x80);
    send(ADDR_P0_SERIAL::BEEP_L_GEN, 0x80|(volume&0x3F)); //enable beep generator with right channel volume,

    //send(ADDR_P0_SERIAL::DAC_NDAC_VAL, 0x84);  //power up NDAC divider - Page 41 (but makes glitches?!)

    send(ADDR_P0_SERIAL::DAC_VOL_CTRL, 0x00); //unmute DACs optional

}
void RvX_TLV320DAC3100::beepMidi(uint8_t midiId, uint16_t lengthMs, bool async) {
    //TODO Check boundaries!
    uint16_t samplerate = 48000; //audioOutputI2S->GetRate();
    int32_t freq = frequencyTable[midiId]; //fixed point /100
    int16_t sin = beepTable16000[midiId][0];
    int16_t cos = beepTable16000[midiId][1];

    switch (samplerate) {
        case 22050:
            sin = beepTable22050[midiId][0];
            cos = beepTable22050[midiId][1];
            break;
        case 32000:
            sin = beepTable32000[midiId][0];
            cos = beepTable32000[midiId][1];
            break;
        case 44100:
            sin = beepTable44100[midiId][0];
            cos = beepTable44100[midiId][1];
            break;
        case 48000:
            sin = beepTable48000[midiId][0];
            cos = beepTable48000[midiId][1];
            break;
}

    int32_t cycles = 2*freq*lengthMs/1000/100;
    int32_t samples_opt = samplerate*(cycles)*100/freq/2;

    //int32_t samples = lengthMs * samplerate / 1000; //check length

    beepRaw(sin, cos, samples_opt);
    if (!async) {
        while ((readByte(ADDR_P0_SERIAL::BEEP_L_GEN) & 0b10000000) == 0b10000000) {
            //Box.watchdog_feed();
            delayTask(1);
        }
    }
}
void RvX_TLV320DAC3100::beep() {
    //beepRaw(0x30FC, 0x7642, 0x640);
    beepMidi(84, 1000, false);
}

uint8_t RvX_TLV320DAC3100::getSampleRateIndex() {
    uint16_t sr = 48000; //audioOutputI2S->GetRate();
    if (sr == 48000) {
        return 4;
    } else if (sr == 44100) {
        return 3;
    } else if (sr == 32000) {
        return 2;
    } else if (sr == 22050) {
        return 1;
    }
    return 0; //16000
}
