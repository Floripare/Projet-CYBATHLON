package com.example.roboticarm;

import androidx.appcompat.app.AppCompatActivity;

import android.content.Intent;
import android.os.Bundle;
import android.view.View;
import android.widget.Button;

public class MainActivity_reglage extends AppCompatActivity {
    //déclaration du bouton de retour
    Button button_retour;

    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        setContentView(R.layout.activity_main_reglage);

        //association et fonction du bouton pour retourner en arrière
        button_retour = findViewById(R.id.button_retour);
        button_retour.setOnClickListener(new View.OnClickListener(){
            @Override
            public void onClick(View v) {
                openActivity_reglage();
            }
        });
    }

    //fonction du bouton pour retourner en arrière
    public void openActivity_reglage(){
        Intent intent = new Intent(this, MainActivity.class);
        startActivity(intent);
    };
}