package com.example.roboticarm;

import androidx.appcompat.app.AppCompatActivity;

import android.annotation.SuppressLint;
import android.content.Context;
import android.content.Intent;
import android.hardware.usb.UsbDeviceConnection;
import android.hardware.usb.UsbManager;
import android.os.Bundle;
import android.view.View;
import android.widget.Button;
import android.widget.SeekBar;
import android.widget.TextView;

import com.hoho.android.usbserial.driver.UsbSerialDriver;
import com.hoho.android.usbserial.driver.UsbSerialPort;
import com.hoho.android.usbserial.driver.UsbSerialProber;

import java.io.IOException;
import java.util.List;
import java.util.InputMismatchException;

import io.github.controlwear.virtual.joystick.android.JoystickView;

public class MainActivity extends AppCompatActivity {

    //déclaration des textView pour les informations du joystick
    private TextView mTextViewAngleRight;
    private TextView mTextViewStrengthRight;
    private TextView mTextViewCoordinateRight;

    //déclaration de la seekbar pour gérer la rotation du poignet
    private SeekBar seekBar_rotation;
    //déclaration du textview pour l'affichage de l'angle du poignet
    private TextView textView_angle;

    //déclaration du bouton pour réinitialiser les réglages
    private Button button_reset;
    private Button button_mode;

    //déclaration du bouton pour accéder aux réglages
    private Button button_reglage;
    String commande = "";
    int mode = 0;
    int reset = 0;

    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        setContentView(R.layout.activity_main);

        // Find all available drivers from attached devices.
        UsbManager manager = (UsbManager) getSystemService(Context.USB_SERVICE);
        List<UsbSerialDriver> availableDrivers = UsbSerialProber.getDefaultProber().findAllDrivers(manager);
        if (availableDrivers.isEmpty()) {
            return;
        }

        // Open a connection to the first available driver.
        UsbSerialDriver driver = availableDrivers.get(0);
        UsbDeviceConnection connection = manager.openDevice(driver.getDevice());
        if (connection == null) {
            // add UsbManager.requestPermission(driver.getDevice(), ..) handling here
            return;
        }

        UsbSerialPort port = driver.getPorts().get(0); // Most devices have just one port (port 0)
        try {
            port.open(connection);
        } catch (IOException e) {
            e.printStackTrace();
        }
        try {
            port.setParameters(115200, 8, UsbSerialPort.STOPBITS_1, UsbSerialPort.PARITY_NONE);
        } catch (IOException e) {
            e.printStackTrace();
        }


        //correspondance avec les textview dans activity_main.xml
        mTextViewAngleRight = (TextView) findViewById(R.id.textView_angle_right);
        mTextViewStrengthRight = (TextView) findViewById(R.id.textView_strength_right);
        mTextViewCoordinateRight = findViewById(R.id.textView_coordinate_right);

        //déclaration du joystick
        final JoystickView joystickRight = (JoystickView) findViewById(R.id.joystickView_right);
        joystickRight.setOnMoveListener(new JoystickView.OnMoveListener() {
            @SuppressLint("DefaultLocale")
            @Override
            public void onMove(int angle, int strength) {
                mTextViewAngleRight.setText(angle + "°");
                mTextViewStrengthRight.setText(strength + "%");
                mTextViewCoordinateRight.setText(
                        String.format("x%03d:y%03d",
                                joystickRight.getNormalizedX(),
                                joystickRight.getNormalizedY())
                );
                commande += "Joystick X = " + strength*Math.cos(angle) + ", Joystick Y =" +  strength*Math.sin(angle);

            }
        });

        //correspondance avec les textview dans activity_main.xml
        seekBar_rotation = findViewById(R.id.seekBar_rotation);
        textView_angle = findViewById(R.id.textView_angle);
        button_reset = findViewById(R.id.button_reset);
        button_mode = findViewById(R.id.button2);

        //fonction pour associer la position de la seekbar avec l'angle affiché
        seekBar_rotation.setOnSeekBarChangeListener(new SeekBar.OnSeekBarChangeListener(){
            @Override
            public void onProgressChanged(SeekBar seekBar, int progress, boolean fromUser) {
                textView_angle.setText(progress+"°");
                commande += ", Wrist Angle is ";
                commande += progress;
            }
            @Override
            public void onStartTrackingTouch(SeekBar seekBar) {

            }
            @Override
            public void onStopTrackingTouch(SeekBar seekBar) {

            }
        });

        button_mode.setOnClickListener(new View.OnClickListener(){
            @Override
            public void onClick(View v) {
                if(mode == 0){
                    mode = 1;
                } else {
                    mode = 0;
                }
                commande += ",Button is ";
                commande += mode;
            }
        });

        //fonction pour reinitialiser la seekbar et l'affichage de l'angle
        button_reset.setOnClickListener(new View.OnClickListener(){
            @Override
            public void onClick(View v) {
                textView_angle.setText("0°");
                seekBar_rotation.setProgress(0);
                if(reset == 0){
                    reset = 1;
                } else {
                    reset = 0;
                }
                commande += ",Reset is ";
                commande += reset;
                System.out.println(commande);
            }
        });

        //association et fonction du bouton pour accéder aux réglages
        button_reglage = findViewById(R.id.button_reglage);
        button_reglage.setOnClickListener(new View.OnClickListener(){
            @Override
            public void onClick(View v) {
                openActivity_reglage();
            }
        });

        commande += ", joystick Max is : 100";
        try {
            port.write(commande.getBytes(), 20);
        } catch (IOException e) {
            e.printStackTrace();
        }


        try {
            port.close();
        } catch (IOException e) {
            e.printStackTrace();
        }
    }

    //fonction pour accéder aux réglages
    public void openActivity_reglage(){
        Intent intent = new Intent(this, MainActivity_reglage.class);
        startActivity(intent);
    };

}