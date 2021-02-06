
package frc.robot;

import java.io.Console;
import java.util.concurrent.DelayQueue;
import java.util.concurrent.Delayed;

import com.ctre.phoenix.CANifier;
import com.ctre.phoenix.CANifier.*;
//import com.ctre.phoenix.CANifier.GeneralPin;

public class Canifier {
    CANifier canifier;

    public Canifier() {
        canifier = new CANifier(1);
    }

    GeneralPin pinArray[] = {GeneralPin.SPI_MISO_PWM2P, GeneralPin.SPI_MOSI_PWM1P, GeneralPin.SPI_CLK_PWM0P, GeneralPin.SPI_CS, GeneralPin.LIMR, GeneralPin.LIMF, GeneralPin.QUAD_IDX, GeneralPin.QUAD_B};
    
    //GeneralPin pinArray[] = {GeneralPin.SPI_MOSI_PWM1P, GeneralPin.SPI_CLK_PWM0P, GeneralPin.SPI_CS, GeneralPin.LIMR, GeneralPin.LIMF, GeneralPin.QUAD_IDX, GeneralPin.QUAD_B, GeneralPin.QUAD_A};

    boolean dataBuf[] = new boolean[8];

    public boolean getPinData()[]{
        for(int i = 0; i < 8; i++){
            dataBuf[i] = canifier.getGeneralInput(pinArray[i]);	
        }
        return dataBuf;
    }

    public String byteArrayToString(boolean array[]){
        String string = "";
        for(int i = 0; i < 8; i++){
            if(array[i]){
                string += "1";
            }
            else{
                string += "0";
            }
        }
        return string;

    }

    public int binToDecCount(boolean arr[]){
        int intBuf = 0;

        for(int ii = 0; ii < 6; ii++){
            if(arr[ii]){
                intBuf += Math.pow(2, ii);
            }
        }
        return intBuf;

    }
    public int binToDecColor(boolean arr[]){
        int colorBuf = 0;
        for(int ii = 6; ii < 8; ii++){
            if(arr[ii]){
                colorBuf += Math.pow(2, (ii-6));
            }
        }
        return colorBuf;
    }

    public void resetPin(boolean on){
        canifier.setGeneralOutput(GeneralPin.QUAD_A, on, true);
    }

}