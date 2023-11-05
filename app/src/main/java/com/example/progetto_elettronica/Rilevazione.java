package com.example.progetto_elettronica;


import static java.util.Arrays.asList;

import android.util.Log;

import java.util.ArrayList;

enum BleObject {
    Temperature,
    Humidity,
    Pressure,
    Inertial,
    Gyro,
    Magnetic,
}

public class Rilevazione {
    private String temperatura;
    private String umidita;
    private String pressione;
    private ArrayList<String> inertia;
    private ArrayList<String>  gyro;
    private ArrayList<String>  magnetic;



    public Rilevazione() {
        gyro= new ArrayList<String>(asList("","",""));
        magnetic= new ArrayList<String>(asList("","",""));
        inertia= new ArrayList<String>(asList("","",""));
    }

    public void commonSetter(BleObject id, String val, int axis)
    {
        if(((id==BleObject.Inertial || id == BleObject.Gyro || id == BleObject.Magnetic) &&
                ( axis < 0 || axis > 2)) || val.contains("Name:") || val.isEmpty() || !val.contains(":"))
            return;
        val = val.replaceAll("\\s", "").split(":")[1];
        Log.i("valore",val+axis);
        switch(id)
        {
            case Temperature:
                setTemperatura(val);
                break;
            case Humidity:
                setUmidita(val);
                break;
            case Pressure:
                setPressione(val);
                break;
            case Gyro:
                setGyro(val, axis);
                break;
            case Inertial:
                setInertia(val, axis);
                break;
            case Magnetic:
                setMagne(val, axis);
                break;
            default:
        }
    }

    public String getTemperatura() {
        return temperatura;
    }

    public String getUmidita() {
        return umidita;
    }

    public String getPressione() {
        return pressione;
    }

    public String getInertia() {
        return inertia.toString();
    }


    public String getGyro() {
        return gyro.toString();
    }

    public String getMagne() {
        return magnetic.toString();
    }

    public void setTemperatura(String temperatura) {
        this.temperatura = temperatura;
    }

    public void setUmidita(String umidita) {
        this.umidita = umidita;
    }

    public void setPressione(String pressione) {
        this.pressione = pressione;
    }

    public void setInertia(String val, int axis) {
        if(axis < 0)
            return;
        inertia.set(axis,val);
    }

    public void setGyro(String val, int axis) {
        if(axis < 0)
            return;
        gyro.set(axis,val);
    }

    public void setMagne(String val, int axis) {
        if(axis < 0)
            return;
        magnetic.set(axis,val);
    }

    public void dump(){
        this.temperatura = "";
        this.umidita = "";
        this.pressione = "";

    }

    @Override
    public String toString() {
        return "Rilevazione{" +
                "temperatura=" + temperatura +
                ", umidita=" + umidita +
                ", pressione=" + pressione +
                ", inertia=" + inertia.toString() +
                ", gyro=" + gyro.toString() +
                ", magnetic=" + magnetic.toString() +
                '}';
    }
}
