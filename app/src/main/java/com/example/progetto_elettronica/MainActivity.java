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




    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        setContentView(R.layout.activity_main);
        manager = getSystemService(BluetoothManager.class);
        bluetoothAdapter = manager.getAdapter();
        bluetoothLeScanner = bluetoothAdapter.getBluetoothLeScanner();
        handler = new Handler();

        final BluetoothManager bluetoothManager = (BluetoothManager) getSystemService(Context.BLUETOOTH_SERVICE);
        bluetoothAdapter = bluetoothManager.getAdapter();
        // Assicurati che il dispositivo supporti BLE
        if (bluetoothAdapter == null || !getPackageManager().hasSystemFeature("android.hardware.bluetooth_le")) {
            // Il dispositivo non supporta BLE
            // Agisci di conseguenza
        }
    }


    private final ScanCallback leScanCallback = new ScanCallback() {
        @Override
        public void onScanResult(int callbackType, ScanResult result) {
            Log.i("risultati",result.getDevice().toString());
            if (!isConnected && result.getDevice().toString().equals("C8:18:76:8B:67:0B")) {
                Log.i("risultati","procede");
                connectToBLEDevice("C8:18:76:8B:67:0B");
            }else if(isConnected){
                Log.i("Conferma","Scansione terminata");
            }else
               Log.i("Valore","nessun interesse");

        }
    };

    public void scanLeDevice(View v) {
        if (!scanning) {
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

    /*
        public void scanLeDevice(View v) {
            if (!scanning) {
                handler.postDelayed(new Runnable() {
                    @Override
                    public void run() {
                        scanning = false;
                        stopScanning();
                    }
                }, SCAN_PERIOD);

                scanning = true;
                startScanning();
            } else {
                scanning = false;
                stopScanning();
            }
        }
    */
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
        Log.i("Successo :","Scansione arestata con successo");
    }




    // Metodo per connettersi a un dispositivo BLE specifico
    private void connectToBLEDevice(String deviceAddress) {
        Log.i("Inizio Connessione","dispositivo "+ deviceAddress);
        BluetoothDevice device = bluetoothAdapter.getRemoteDevice(deviceAddress);
        if (ActivityCompat.checkSelfPermission(this, Manifest.permission.BLUETOOTH_CONNECT) != PackageManager.PERMISSION_GRANTED) {
            Log.i("Esito","errato, niente permessi");
            return;
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
            Log.i("Connessione In corso","Fin qui tutto nella norma");
            if (newState == BluetoothProfile.STATE_CONNECTED) {
                Log.i("Connesso", "Connessione avvenuta con successo");
                if (ActivityCompat.checkSelfPermission(MainActivity.this, Manifest.permission.BLUETOOTH_CONNECT) == PackageManager.PERMISSION_GRANTED) {
                    isConnected = true;
                    if (ActivityCompat.checkSelfPermission(MainActivity.this, Manifest.permission.BLUETOOTH_SCAN) == PackageManager.PERMISSION_GRANTED) {

                    }
                    bluetoothLeScanner.stopScan(leScanCallback);
                    //gatt.discoverServices();
                }
            } else if (newState == BluetoothProfile.STATE_DISCONNECTED) {
                Log.i("Errore", "Connessione non avvenuta con successo");
                gatt.close();
                isConnected = false;
                startScanning();
            }

        }

        // Altre callback per le operazioni BLE, come la lettura e la scrittura dei dati


        // Dopo che la connessione è stata stabilita con successo


        @Override
        public void onServicesDiscovered(BluetoothGatt gatt, int status) {
            Log.i("Servizii :","trovati");
            if (status == BluetoothGatt.GATT_SUCCESS) {
                List<BluetoothGattService> services = gatt.getServices();
                List<BluetoothGattCharacteristic> allCharacteristics = new ArrayList<>();
                Log.i("Servizi :","Raccolti");
                for (BluetoothGattService service : services) {
                    List<BluetoothGattCharacteristic> characteristics = service.getCharacteristics();
                    allCharacteristics.addAll(characteristics);
                    Log.i("Caratteristiche :","Raccolte");
                }

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
                    Log.i("Lettura.Caratteristiche :","In corso");
                    if (counter < characteristics.size()) {
                        BluetoothGattCharacteristic characteristic = characteristics.get(counter);
                        if (ActivityCompat.checkSelfPermission(MainActivity.this, Manifest.permission.BLUETOOTH_CONNECT) == PackageManager.PERMISSION_GRANTED) {
                            gatt.readCharacteristic(characteristic);
                            counter=0;
                        }
                    } else {
                        // Se hai letto tutte le caratteristiche, ripeti il ciclo
                        counter=0;
                    }
                    // Richiama il runnable con il ritardo specificato
                    handler.postDelayed(this, delay);
                }
            };

            // Avvia il ciclo
            handler.postDelayed(runnable, delay);
        }


    };
}