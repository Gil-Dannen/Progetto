package com.example.progetto_elettronica;


import androidx.appcompat.app.AppCompatActivity;
import androidx.core.app.ActivityCompat;

import android.Manifest;

import android.bluetooth.BluetoothAdapter;

import android.bluetooth.BluetoothDevice;
import android.bluetooth.BluetoothGatt;
import android.bluetooth.BluetoothGattCallback;
import android.bluetooth.BluetoothGattCharacteristic;
import android.bluetooth.BluetoothGattService;
import android.bluetooth.BluetoothManager;
import android.bluetooth.BluetoothProfile;
import android.bluetooth.le.BluetoothLeScanner;
import android.bluetooth.le.ScanCallback;
import android.bluetooth.le.ScanResult;

import android.content.Context;

import android.content.pm.PackageManager;

import android.os.Build;
import android.os.Bundle;
import android.os.Handler;

import android.util.Log;
import android.view.View;
import android.widget.Button;
import android.widget.LinearLayout;
import android.widget.TextView;


import java.util.ArrayList;

import java.util.List;


public class MainActivity extends AppCompatActivity {
    private BluetoothGatt bluetoothGatt;
    private BluetoothManager manager;
    private BluetoothAdapter bluetoothAdapter;
    private BluetoothLeScanner bluetoothLeScanner;
    private boolean scanning;
    private boolean isConnected=false;
    private Handler handler = new Handler();
    private int counter=0;
    // Stops scanning after 10 seconds.
    private static final long SCAN_PERIOD = 10000;
    public static final int MY_PERMISSION = 100;
    private Rilevazione rilevazione;
    private String current;
    private Button button;
    private LinearLayout testo;
    private boolean visible=false;
    private HashMap<BleObject,String> mapHelper;
    private HashMap<Integer,String> axisMapHelper;
    private BleObject objTemp;
    private int tempAxis;


    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        setContentView(R.layout.activity_main);
        manager = getSystemService(BluetoothManager.class);
        bluetoothAdapter = manager.getAdapter();
        bluetoothLeScanner = bluetoothAdapter.getBluetoothLeScanner();
        handler = new Handler();
        rilevazione= new Rilevazione();
        button = findViewById(R.id.Bottone);
        testo=(LinearLayout) findViewById(R.id.Layout);

        final BluetoothManager bluetoothManager = (BluetoothManager) getSystemService(Context.BLUETOOTH_SERVICE);
        bluetoothAdapter = bluetoothManager.getAdapter();
        mapHelper = new HashMap<BleObject,String>();
        mapHelper.put(BleObject.Temperature,"Temperature");
        mapHelper.put(BleObject.Humidity,"Humidity");
        mapHelper.put(BleObject.Pressure,"Pressure");
        mapHelper.put(BleObject.Inertial,"Inertia");
        mapHelper.put(BleObject.Magnetic,"magnetic");
        mapHelper.put(BleObject.Gyro,"gyroscope");
        axisMapHelper = new HashMap<Integer ,String>();
        axisMapHelper.put(0,"X");
        axisMapHelper.put(1,"Y");
        axisMapHelper.put(2,"Z");

    }


    private final ScanCallback leScanCallback = new ScanCallback() {
        @Override
        public void onScanResult(int callbackType, ScanResult result) {
           // Log.i("risultati",result.getDevice().toString());
            if (!isConnected && result.getDevice().toString().equals("C8:18:76:8B:67:0B")) {
                //Log.i("risultati","procede");
                connectToBLEDevice();
            }/*else if(isConnected){
                //Log.i("Conferma","Scansione terminata");
            }else;
               //Log.i("Valore","nessun interesse");
               */

        }
    };

    public void scanLeDevice(View v) {
        button.setText("Ricerca");
        if (!scanning || !isConnected) {
            // Stops scanning after a predefined scan period.
            handler.postDelayed(new Runnable() {
                @Override
                public void run() {
                    scanning = false;
                    if (ActivityCompat.checkSelfPermission(MainActivity.this, Manifest.permission.BLUETOOTH_SCAN) != PackageManager.PERMISSION_GRANTED) {
                        if (Build.VERSION.SDK_INT >= 31) {
                            ActivityCompat.requestPermissions(MainActivity.this, new String[]{Manifest.permission.BLUETOOTH_SCAN}, 100);
                            return;
                        }
                    }
                    bluetoothLeScanner.stopScan(leScanCallback);
                }
            }, SCAN_PERIOD);

            scanning = true;
            if (ActivityCompat.checkSelfPermission(this, Manifest.permission.BLUETOOTH_SCAN) != PackageManager.PERMISSION_GRANTED) {
                if (Build.VERSION.SDK_INT >= 31) {
                    ActivityCompat.requestPermissions(MainActivity.this, new String[]{Manifest.permission.BLUETOOTH_SCAN}, 100);
                    return;
                }
            }
            bluetoothLeScanner.startScan(leScanCallback);
        } else {
            scanning = false;
            if (ActivityCompat.checkSelfPermission(this, Manifest.permission.BLUETOOTH_SCAN) != PackageManager.PERMISSION_GRANTED) {
                return;
            }
            bluetoothLeScanner.stopScan(leScanCallback);
        }
    }
    private void startScanning() {
        if (ActivityCompat.checkSelfPermission(this, Manifest.permission.BLUETOOTH_SCAN) == PackageManager.PERMISSION_GRANTED) {
            if (Build.VERSION.SDK_INT >= 31) {
                ActivityCompat.requestPermissions(this, new String[]{Manifest.permission.BLUETOOTH_SCAN}, 100);
                return;
            }

        }
        bluetoothLeScanner.startScan(leScanCallback);
    }

    private void stopScanning() {
        if (ActivityCompat.checkSelfPermission(this, Manifest.permission.BLUETOOTH_SCAN) == PackageManager.PERMISSION_GRANTED) {
            Log.e("Errore :","Posizione non gradita");
        }
        bluetoothLeScanner.stopScan(leScanCallback);
        //Log.i("Successo :","Scansione arestata con successo");
    }

    private void updateView(){
        TextView temperatura =(TextView) findViewById(R.id.tempe);
        TextView pressione =(TextView) findViewById(R.id.pres);
        TextView umidita =(TextView) findViewById(R.id.hum);
        TextView magne =(TextView) findViewById(R.id.magne);
        TextView inerthia =(TextView) findViewById(R.id.iner);
        TextView gyro =(TextView)  findViewById(R.id.gyro);
        temperatura.setText(rilevazione.getTemperatura());
        pressione.setText(rilevazione.getPressione());
        umidita.setText(rilevazione.getUmidita());
        magne.setText(rilevazione.getMagne());
        inerthia.setText(rilevazione.getInertia());
        gyro.setText(rilevazione.getGyro());
    }




    // Metodo per connettersi a un dispositivo BLE specifico
    private void connectToBLEDevice() {
        //Log.i("Inizio Connessione","dispositivo "+ deviceAddress);
        BluetoothDevice device = bluetoothAdapter.getRemoteDevice("C8:18:76:8B:67:0B");
        if (ActivityCompat.checkSelfPermission(this, Manifest.permission.BLUETOOTH_CONNECT) != PackageManager.PERMISSION_GRANTED) {
           // Log.i("Esito","errato, niente permessi");
            ActivityCompat.requestPermissions(this,new String[]{
                    Manifest.permission.BLUETOOTH_CONNECT
            },MY_PERMISSION);
        }
        if (bluetoothGatt != null) {
            bluetoothGatt.close();
            bluetoothGatt = null;
        }
        bluetoothGatt = device.connectGatt(this, false, gattCallback);

    }
    // Callback per la connessione al dispositivo BLE

    private final BluetoothGattCallback gattCallback = new BluetoothGattCallback() {
        @Override
        public void onConnectionStateChange(BluetoothGatt gatt, int status, int newState) {
           // Log.i("Connessione In corso","Fin qui tutto nella norma");
            if (newState == BluetoothProfile.STATE_CONNECTED) {
                Log.i("Connesso", "Connessione avvenuta con successo");
                if (ActivityCompat.checkSelfPermission(MainActivity.this, Manifest.permission.BLUETOOTH_CONNECT) == PackageManager.PERMISSION_GRANTED) {
                    isConnected = true;
                    if (ActivityCompat.checkSelfPermission(MainActivity.this, Manifest.permission.BLUETOOTH_SCAN) == PackageManager.PERMISSION_GRANTED) {
                        //Log.e("Errore", "Connessione non avvenuta con successo");
                    }
                    stopScanning();
                    gatt.discoverServices();

                }
            } else if (newState == BluetoothProfile.STATE_DISCONNECTED) {
               //Log.i("Errore", "Connessione non avvenuta con successo");
                gatt.close();
                isConnected = false;
                startScanning();
            }

        }

        // Altre callback per le operazioni BLE, come la lettura e la scrittura dei dati


        // Dopo che la connessione è stata stabilita con successo


        @Override
        public void onCharacteristicRead (BluetoothGatt gatt, BluetoothGattCharacteristic characteristic,int status) {
            //byte[] data = characteristic.getValue();
            String dataString = characteristic.getStringValue(0);
           // Log.i("BLE", "Dati letti: " + dataString);

            try {

                mapHelper.forEach((bleObject, s) ->
                {
                    if(dataString.contains(s))
                        objTemp = bleObject;
                });

                tempAxis = -1;

                axisMapHelper.forEach((i, s) ->
                {
                    if(dataString.contains(s))
                        tempAxis = i;
                });

                rilevazione.commonSetter(objTemp,dataString.replaceAll("\\s", "").split(":")[1],tempAxis);


           }catch(Exception ex){
               Log.e("Errore",ex.toString());
           }
        }

        @Override
        public void onServicesDiscovered(BluetoothGatt gatt, int status) {
           // Log.i("Servizi :","trovati");
            if (status == BluetoothGatt.GATT_SUCCESS) {
                List<BluetoothGattService> services = gatt.getServices();
                List<BluetoothGattCharacteristic> allCharacteristics = new ArrayList<>();
               // Log.i("Servizi :","Raccolti");
                //int conta =0;
                for (BluetoothGattService service : services) {
                    List<BluetoothGattCharacteristic> characteristics = service.getCharacteristics();
/*
                    for(BluetoothGattCharacteristic charateristic : characteristics){
                        Log.i("Caratteristica ", charateristic.getValue()+" "+conta++);

                    }

 */
                    allCharacteristics.addAll(characteristics);
                }
               // Log.i("Caratteristiche :","Raccolte");

                // Inizia il ciclo di lettura continua delle caratteristiche
                readCharacteristicsInLoop(gatt, allCharacteristics);
            } else {
                Log.e("Errore", "Scoperta dei servizi fallita");
            }
        }


        // Funzione per leggere continuamente le caratteristiche in un ciclo
        private void readCharacteristicsInLoop(final BluetoothGatt gatt, final List<BluetoothGattCharacteristic> characteristics) {

            final int delay = 1000; // Ritardo in millisecondi tra le letture

            final Runnable runnable = new Runnable() {
                @Override
                public void run() {
                    if (ActivityCompat.checkSelfPermission(MainActivity.this, Manifest.permission.BLUETOOTH_CONNECT) != PackageManager.PERMISSION_GRANTED)
                        return;

                    Log.i("Lettura.Caratteristiche :","In corso");
                    if (counter < characteristics.size()) {

                        try {
                            BluetoothGattCharacteristic characteristic = characteristics.get(counter);
                            gatt.readCharacteristic(characteristic);

                            counter++;
                        }catch(Exception errore){
                            Log.e("Catch",errore.toString());
                        }
                    } else {
                        // Se hai letto tutte le caratteristiche, ripeti il ciclo
                        //Log.i("Stringa",rilevazione.toString());
                        updateView();
                        if(!visible) {
                            testo.setVisibility(View.VISIBLE);
                            button.setVisibility(View.GONE);
                        }
                        counter=0;
                        rilevazione.dump();
                    }
                    // Richiama il runnable con il ritardo specificato
                    handler.postDelayed(this, delay);
                }
            };

            // Avvia il ciclo
            handler.postDelayed(runnable, delay);
        }


    };


    @Override
    public void onRequestPermissionsResult(int requestCode,String permissions[], int[] grantResults) {
        super.onRequestPermissionsResult(requestCode, permissions, grantResults);
        switch (requestCode) {
            case MY_PERMISSION: {
                // Se la richiesta è stata cancellata, l'array result è vuoto
                if (grantResults.length > 0 && grantResults[0] == PackageManager.PERMISSION_GRANTED) {
                    // utilizza permesso
                } else {
                    // gestisci permesso negato
                }

            }
            // altri casi
        }
    }
}