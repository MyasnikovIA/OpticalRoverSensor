package ru.miacomsoft.optical_rover_sensor;

import android.Manifest;
import android.annotation.SuppressLint;
import android.content.Intent;
import android.content.pm.PackageManager;
import android.os.Build;
import android.os.Bundle;
import android.os.Handler;
import android.os.Looper;
import android.view.View;
import android.webkit.WebSettings;
import android.webkit.WebView;
import android.webkit.WebViewClient;
import android.widget.Button;
import android.widget.LinearLayout;
import android.widget.TextView;

import androidx.annotation.NonNull;
import androidx.appcompat.app.AppCompatActivity;
import androidx.core.app.ActivityCompat;
import androidx.core.content.ContextCompat;

public class MainActivity extends AppCompatActivity {

    private WebView webLive;
    private WebView webSettings;
    private Button btnToggle;
    private LinearLayout loadingPanel;
    private TextView loadingText;
    private Button grantPermBtn;

    private boolean serverStarted = false;
    private boolean isLiveView = true;
    private final Handler handler = new Handler(Looper.getMainLooper());
    private Runnable pingRunnable;

    @SuppressLint("SetJavaScriptEnabled")
    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        setContentView(R.layout.activity_main);

        webLive = findViewById(R.id.webLive);
        webSettings = findViewById(R.id.webSettings);
        btnToggle = findViewById(R.id.btnToggle);
        loadingPanel = findViewById(R.id.loadingPanel);
        loadingText = findViewById(R.id.loadingText);
        grantPermBtn = findViewById(R.id.grantPermBtn);

        setupWebViews();

        btnToggle.setOnClickListener(v -> toggleView());
        grantPermBtn.setOnClickListener(v -> requestCameraPermission());

        if (hasCameraPermission()) {
            grantPermBtn.setVisibility(View.GONE);
            startSensorService();
        } else {
            loadingText.setText("Требуется разрешение на камеру");
            grantPermBtn.setVisibility(View.VISIBLE);
        }
    }

    private void setupWebViews() {
        WebSettings liveSettings = webLive.getSettings();
        liveSettings.setJavaScriptEnabled(true);
        liveSettings.setDomStorageEnabled(true);
        liveSettings.setDatabaseEnabled(false); // Убираем ошибку sqlite в логах на старых API
        liveSettings.setLoadWithOverviewMode(true);
        liveSettings.setUseWideViewPort(true);
        webLive.setWebViewClient(new WebViewClient());

        WebSettings setSettings = webSettings.getSettings();
        setSettings.setJavaScriptEnabled(true);
        setSettings.setDomStorageEnabled(true);
        setSettings.setDatabaseEnabled(false);
        setSettings.setLoadWithOverviewMode(true);
        setSettings.setUseWideViewPort(true);
        webSettings.setWebViewClient(new WebViewClient());
    }

    private boolean hasCameraPermission() {
        if (Build.VERSION.SDK_INT >= Build.VERSION_CODES.M) {
            return ContextCompat.checkSelfPermission(this, Manifest.permission.CAMERA) == PackageManager.PERMISSION_GRANTED;
        }
        return true;
    }

    private void requestCameraPermission() {
        if (Build.VERSION.SDK_INT >= Build.VERSION_CODES.M) {
            ActivityCompat.requestPermissions(this, new String[]{Manifest.permission.CAMERA}, 100);
        }
    }

    @Override
    public void onRequestPermissionsResult(int requestCode, @NonNull String[] permissions, @NonNull int[] grantResults) {
        super.onRequestPermissionsResult(requestCode, permissions, grantResults);
        if (requestCode == 100) {
            if (grantResults.length > 0 && grantResults[0] == PackageManager.PERMISSION_GRANTED) {
                grantPermBtn.setVisibility(View.GONE);
                startSensorService();
            } else {
                loadingText.setText("Разрешение на камеру отклонено.");
            }
        }
    }

    private void startSensorService() {
        loadingText.setText("Запуск сервиса…");

        // Запуск фонового сервиса
        Intent intent = new Intent(this, SensorService.class);
        intent.setAction(SensorService.ACTION_START);
        if (Build.VERSION.SDK_INT >= Build.VERSION_CODES.O) {
            startForegroundService(intent);
        } else {
            startService(intent);
        }

        checkServerStatus();
    }

    private void checkServerStatus() {
        if (pingRunnable != null) handler.removeCallbacks(pingRunnable);

        // Читаем порт из конфига (по умолчанию 8080)
        int port = 8080;
        try {
            port = ConfigStore.load(this).httpPort;
        } catch (Exception ignored) {}

        final int finalPort = port;

        pingRunnable = new Runnable() {
            @Override
            public void run() {
                if (serverStarted) return;

                // Пытаемся подключиться к локальному серверу
                new Thread(() -> {
                    try {
                        java.net.Socket socket = new java.net.Socket();
                        socket.connect(new java.net.InetSocketAddress("127.0.0.1", finalPort), 500);
                        socket.close();
                        // Если подключение успешно, сервер запущен
                        handler.post(() -> onServerStarted(finalPort));
                    } catch (Exception e) {
                        // Ждем 1 секунду и пробуем снова
                        handler.postDelayed(pingRunnable, 1000);
                    }
                }).start();
            }
        };

        handler.postDelayed(pingRunnable, 1000);
    }

    private void onServerStarted(int port) {
        if (serverStarted) return;
        serverStarted = true;

        // Скрываем экран загрузки, показываем кнопки
        loadingPanel.setVisibility(View.GONE);
        btnToggle.setVisibility(View.VISIBLE);

        // Загружаем локальные страницы в WebView
        webLive.loadUrl("http://127.0.0.1:" + port + "/sensor.html");
        webSettings.loadUrl("http://127.0.0.1:" + port + "/config.html");

        // Показываем Live по умолчанию
        isLiveView = true;
        applyVisibility();
    }

    private void toggleView() {
        isLiveView = !isLiveView;
        applyVisibility();
    }

    private void applyVisibility() {
        if (isLiveView) {
            webLive.setVisibility(View.VISIBLE);
            webSettings.setVisibility(View.GONE);
            btnToggle.setText("⚙");
        } else {
            webLive.setVisibility(View.GONE);
            webSettings.setVisibility(View.VISIBLE);
            btnToggle.setText("✕");
        }
    }

    @Override
    protected void onDestroy() {
        super.onDestroy();
        if (pingRunnable != null) handler.removeCallbacks(pingRunnable);
    }
}