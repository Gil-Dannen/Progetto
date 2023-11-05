package com.example.progetto_elettronica;

public class Rilevazione {
    private String temperatura;
    private String umidita;
    private String pressione;
    private String inertiaX;
    private String inertiaY;
    private String inertiaZ;
    private String gyroX;
    private String gyroY;
    private String gyroZ;
    private String magneX;
    private String magneY;
    private String magneZ;



    public Rilevazione() {
        
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
        String val="Accelerometro :   AsseX="+inertiaX+" AsseY="+ inertiaY + " AsseZ="+ inertiaZ;
        return val;
    }


    public String getGyro() {
        String val="Giroscopio :   AsseX="+gyroX+" AsseY="+ gyroY + " AsseZ="+ gyroZ;
        return val;
    }

    public String getMagne() {
        String val="Magnetometro :   AsseX="+magneX +" AsseY="+ magneY + " AsseZ="+ magneZ;
        return val;
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

    public void setInertiaX(String inertiaX) {
        this.inertiaX = inertiaX;
    }

    public void setInertiaY(String inertiaY) {
        this.inertiaY = inertiaY;
    }

    public void setInertiaZ(String inertiaZ) {
        this.inertiaZ = inertiaZ;
    }

    public void setGyroX(String gyroX) {
        this.gyroX = gyroX;
    }

    public void setGyroY(String gyroY) {
        this.gyroY = gyroY;
    }

    public void setGyroZ(String gyroZ) {
        this.gyroZ = gyroZ;
    }

    public void setMagneX(String magneX) {
        this.magneX = magneX;
    }

    public void setMagneY(String magneY) {
        this.magneY = magneY;
    }

    public void setMagneZ(String magneZ) {
        this.magneZ = magneZ;
    }

    public void dump(){
        this.temperatura = "";
        this.umidita = "";
        this.pressione = "";
        this.inertiaX = "";
        this.inertiaY = "";
        this.inertiaZ = "";
        this.gyroX = "";
        this.gyroY = "";
        this.gyroZ = "";
        this.magneX = "";
        this.magneY = "";
        this.magneZ = "";
    }

    @Override
    public String toString() {
        return "Rilevazione{" +
                "temperatura=" + temperatura +
                ", umidita=" + umidita +
                ", pressione=" + pressione +
                ", inertiaX=" + inertiaX +
                ", inertiaY=" + inertiaY +
                ", inertiaZ=" + inertiaZ +
                ", gyroX=" + gyroX +
                ", gyroY=" + gyroY +
                ", gyroZ=" + gyroZ +
                ", magneX=" + magneX +
                ", magneY=" + magneY +
                ", magneZ=" + magneZ +
                '}';
    }
}
