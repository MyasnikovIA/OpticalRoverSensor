package ru.miacomsoft.optical_rover_sensor;

import android.app.Notification;
import android.app.NotificationChannel;
import android.app.NotificationManager;
import android.app.PendingIntent;
import android.app.Service;
import android.content.Context;
import android.content.Intent;
import android.graphics.ImageFormat;
import android.graphics.SurfaceTexture;
import android.hardware.Camera;
import android.os.Build;
import android.os.Handler;
import android.os.HandlerThread;
import android.os.IBinder;
import android.os.Looper;
import android.util.Log;

import androidx.annotation.Nullable;
import androidx.core.app.NotificationCompat;

import org.json.JSONObject;

import java.text.SimpleDateFormat;
import java.util.ArrayList;
import java.util.Collections;
import java.util.Comparator;
import java.util.Date;
import java.util.List;
import java.util.Locale;
import java.util.concurrent.atomic.AtomicBoolean;

public class SensorService extends Service {

    private static final String TAG = "SensorService";
    private static final String CHANNEL_ID = "sensor_channel";
    private static final int    NOTIF_ID   = 2001;

    public static final String ACTION_START = "ru.miacomsoft.optical_rover_sensor.START";
    public static final String ACTION_STOP  = "ru.miacomsoft.optical_rover_sensor.STOP";

    // ---- Fallback: накопленный поворот по block matching (пока нет калибровки) ----
    private volatile double totalRotationDeg = 0.0;

    // ---- Runtime: ноль угла (posY, при котором угол = 0) ----
    private double rotZeroPx = 0.0;

    // ---- Калибровка поворота (runtime) ----
    private volatile boolean rotCalibActive    = false;
    private volatile double  rotCalibStartPosY = 0.0;
    private volatile long    rotCalibStartMs   = 0L;

    // ---- Мировая позиция ровера (пересчитывается из forward + rotation) ----
    private double worldX = 0.0;
    private double worldY = 0.0;
    private double zeroWorldX = 0.0;
    private double zeroWorldY = 0.0;

    // ---- Трейл (мировые координаты) ----
    private final Object trailLock = new Object();
    private final List<double[]> trail = new ArrayList<double[]>(); // {worldX, worldY, rotDeg, t_ms}
    private static final int    MAX_TRAIL_POINTS = 1000;
    private static final double TRAIL_MIN_DIST   = 1.0;
    private double lastTrailX = 0.0, lastTrailY = 0.0;

    // ---------- HTTP port change notification ----------
    public interface HttpPortListener {
        void onHttpPortChanged(int requestedPort, int actualPort);
    }
    private static volatile HttpPortListener sPortListener;
    private static volatile int sCurrentHttpPort = 8080;

    public static int getCurrentHttpPort() { return sCurrentHttpPort; }
    public static void setHttpPortListener(HttpPortListener l) { sPortListener = l; }

    private static void notifyPortChanged(int requestedPort, int actualPort) {
        HttpPortListener l = sPortListener;
        if (l != null) {
            try { l.onHttpPortChanged(requestedPort, actualPort); }
            catch (Exception ignored) {}
        }
    }

    // ---------- state ----------
    private SensorConfig config = new SensorConfig();
    private final SensorBroadcaster broadcaster = new SensorBroadcaster();

    private Camera camera;
    private SurfaceTexture dummyTexture;
    private int previewWidth = 0, previewHeight = 0;
    private byte[] yPlane;
    private final AtomicBoolean frameBusy = new AtomicBoolean(false);

    private volatile byte[] fullFrame;
    private final Object    snapshotLock = new Object();
    private SnapshotProvider snapshotProvider;

    private HandlerThread worker;
    private Handler workerHandler;

    private Handler uiHandler;
    private Runnable uiTick;
    private static final long UI_TICK_MS = 15000L;

    private OpticalFlowTracker tracker = new OpticalFlowTracker();
    private FlashlightController flashlight;

    private HttpApiServer   httpServer;
    private WebSocketServer wsServer;
    private TelnetServer    telnetServer;
    private PushClient      pushClient;

    private volatile boolean sensorRunning = false;
    private volatile String  lastCameraError = "";
    private long seq = 0;
    private long lastTimestamp = 0;

    // ---- publish filter ----
    private long   lastPublishMs   = 0L;
    private String lastFlashState  = "";
    private long   skippedReadings = 0L;
    private volatile OpticalFlowTracker.Result lastFlowResult;
    private final Object flowResultLock = new Object();

    private final Object positionLock = new Object();
    private double posX = 0.0, posY = 0.0;  // posX = forward acc, posY = rot acc (px)
    private double zeroX = 0.0, zeroY = 0.0;
    private long   zeroSetAtMs = 0L;
    private volatile boolean forcePublish  = false;
    private volatile long    settleUntilMs = 0L;

    private final SimpleDateFormat iso = new SimpleDateFormat(
            "yyyy-MM-dd'T'HH:mm:ss.SSSZ", Locale.US);

    // ---- Калибровка покоя ----
    private final Object calibLock = new Object();
    private final List<double[]> calibSamples = new ArrayList<>();
    private volatile boolean calibActive = false;
    private volatile boolean calibPendingZero = false;
    private volatile long    calibStartMs = 0;
    private volatile String  calibStatus = "idle";
    private static final int    CALIB_MIN_SAMPLES    = 30;
    private static final int    CALIB_MAX_MULTIPLIER = 3;
    private static final double CALIB_REJECT_PX      = 5.0;
    private volatile long calibTargetMs = 0L;

    private volatile org.json.JSONArray cachedCameraSizes = null;

    // ==============================================================
    // Lifecycle
    // ==============================================================

    @Override
    public void onCreate() {
        super.onCreate();
        createChannel();
        uiHandler = new Handler(Looper.getMainLooper());
    }

    @Override
    public int onStartCommand(Intent intent, int flags, int startId) {
        String action = intent == null ? null : intent.getAction();

        if (ACTION_STOP.equals(action)) {
            shutdown();
            stopSelf();
            return START_NOT_STICKY;
        }

        startForeground(NOTIF_ID, buildNotification("Запуск…"));

        try {
            config = ConfigStore.load(getApplicationContext());
            Log.i(TAG, "Config loaded: enabled=" + config.enabled
                    + ", autoStart=" + config.autoStart
                    + ", serverEnabled=" + config.serverEnabled
                    + ", httpPort=" + config.httpPort
                    + ", flashMode=" + config.flashlightMode
                    + ", rotCalibDone=" + config.rotCalibDone
                    + ", pxPerDeg=" + config.rotCalibPxPerDeg);
        } catch (Exception e) {
            Log.e(TAG, "config load", e);
        }

        snapshotProvider = new SnapshotProvider(this,
                config.cameraPreviewFps, config.cameraPreviewQuality);

        if (config.enabled && config.serverEnabled) startServers();
        if (config.enabled && config.autoStart)     startSensorOnly();
        if (config.enabled && config.clientEnabled) startClient();

        updateNotification(buildStatusLine());

        if (uiTick == null) {
            uiTick = new Runnable() {
                @Override public void run() {
                    updateNotification(buildStatusLine());
                    if (uiHandler != null) uiHandler.postDelayed(this, UI_TICK_MS);
                }
            };
            uiHandler.postDelayed(uiTick, UI_TICK_MS);
        }

        return START_STICKY;
    }

    @Override
    public void onDestroy() {
        shutdown();
        super.onDestroy();
    }

    @Nullable @Override public IBinder onBind(Intent intent) { return null; }

    // ==============================================================
    // Config / status
    // ==============================================================

    public SensorConfig getConfig() { return config; }
    public SensorBroadcaster getBroadcaster() { return broadcaster; }

    /**
     * Текущее положение становится точкой отсчёта (0,0) в мировой системе.
     * Угол НЕ сбрасывается (сохраняем текущий курс).
     */
    public synchronized void setZero() {
        synchronized (positionLock) {
            zeroWorldX = worldX;
            zeroWorldY = worldY;
            // old-поля — для обратной совместимости, не влияют на карту
            zeroX = posX;
            zeroY = posY;
            zeroSetAtMs = System.currentTimeMillis();
        }
        settleUntilMs = System.currentTimeMillis() + 300;
        forcePublish  = true;
        Log.i(TAG, "Zero set: world=(" + zeroWorldX + ", " + zeroWorldY + "), settle 300ms");
    }

    /**
     * Полный сброс: позиция, поворот, трейл, ноль.
     */
    public synchronized void resetPosition() {
        synchronized (positionLock) {
            posX = 0.0; posY = 0.0;
            zeroX = 0.0; zeroY = 0.0;
            worldX = 0.0; worldY = 0.0;
            zeroWorldX = 0.0; zeroWorldY = 0.0;
            rotZeroPx = 0.0;
            zeroSetAtMs = System.currentTimeMillis();
        }
        totalRotationDeg = 0.0;
        synchronized (trailLock) {
            trail.clear();
            trail.add(new double[]{0, 0, 0, System.currentTimeMillis()});
            lastTrailX = 0;
            lastTrailY = 0;
        }
        settleUntilMs = System.currentTimeMillis() + 500;
        forcePublish  = true;
        Log.i(TAG, "Position and rotation reset (settle 500ms)");
    }

    /**
     * Текущий угол поворота в градусах (не нормализован, может быть > 360).
     * Если калибровка не выполнена — возвращает block-matching fallback.
     */
    private double computeRotationDeg() {
        if (config.rotCalibDone && Math.abs(config.rotCalibPxPerDeg) > 1e-9) {
            return (posY - rotZeroPx) / config.rotCalibPxPerDeg;
        }
        // Временный fallback: используем накопленный X-поток (posY).
        // При правом повороте posY уменьшается, поэтому инвертируем знак,
        // чтобы правый поворот давал положительный угол (по часовой стрелке).
        return -(posY - rotZeroPx);
    }

    // ==============================================================
    // Калибровка поворота (X-поток → угол)
    // ==============================================================

    public synchronized void startRotationCalib() {
        if (!sensorRunning) {
            Log.w(TAG, "startRotationCalib: sensor not running");
            return;
        }
        rotCalibStartPosY = posY;
        rotCalibStartMs   = System.currentTimeMillis();
        rotCalibActive    = true;
        Log.i(TAG, "Rotation calibration STARTED, startPosY=" + rotCalibStartPosY);
    }

    /**
     * Завершает калибровку. angleDeg — угол, на который пользователь
     * повернул устройство (положительный = по часовой стрелке, если так удобно).
     * Запоминает знаковый коэффициент px/°.
     */
    public synchronized boolean finishRotationCalib(double angleDeg) {
        if (!rotCalibActive) {
            Log.w(TAG, "finishRotationCalib: not active");
            return false;
        }
        if (Math.abs(angleDeg) < 1e-3) {
            Log.w(TAG, "finishRotationCalib: angle too small");
            rotCalibActive = false;
            return false;
        }
        double delta = posY - rotCalibStartPosY;
        if (Math.abs(delta) < 1e-6) {
            Log.w(TAG, "finishRotationCalib: no X motion detected");
            rotCalibActive = false;
            return false;
        }
        config.rotCalibPxPerDeg = delta / angleDeg;
        config.rotCalibDone     = true;
        rotZeroPx               = rotCalibStartPosY;
        rotCalibActive          = false;

        try { ConfigStore.save(getApplicationContext(), config); }
        catch (Exception e) { Log.w(TAG, "save rotCalib", e); }

        Log.i(TAG, "Rotation calibration DONE: deltaPx=" + delta
                + ", angle=" + angleDeg + "°, pxPerDeg=" + config.rotCalibPxPerDeg
                + ", rotZeroPx=" + rotZeroPx);

        forcePublish = true;
        return true;
    }

    public synchronized void cancelRotationCalib() {
        rotCalibActive = false;
        Log.i(TAG, "Rotation calibration CANCELLED");
    }

    public synchronized void clearRotationCalib() {
        config.rotCalibDone     = false;
        config.rotCalibPxPerDeg = 0.0;
        rotZeroPx               = 0.0;
        rotCalibActive          = false;
        try { ConfigStore.save(getApplicationContext(), config); }
        catch (Exception e) { Log.w(TAG, "save clear rotCalib", e); }
        Log.i(TAG, "Rotation calibration CLEARED");
    }

    public JSONObject rotationCalibJson() {
        try {
            JSONObject o = new JSONObject();
            o.put("active",     rotCalibActive);
            o.put("calibrated", config.rotCalibDone);
            o.put("pxPerDeg",   config.rotCalibPxPerDeg);
            o.put("rotZeroPx",  rotZeroPx);
            o.put("startPosY",  rotCalibStartPosY);
            o.put("currentPosY", posY);
            o.put("deltaPx",    posY - rotCalibStartPosY);
            o.put("currentRotationDeg",
                    round(normalizeDeg(computeRotationDeg()), 2));
            o.put("elapsedMs",
                    rotCalibActive ? System.currentTimeMillis() - rotCalibStartMs : 0);
            return o;
        } catch (Exception e) { return new JSONObject(); }
    }

    // ==============================================================
    // Калибровка покоя (position deadband) — без изменений
    // ==============================================================

    public synchronized void startCalibStatic(boolean thenZero) {
        startCalibStatic(thenZero, config.calibDurationSec);
    }

    public synchronized void startCalibStatic(boolean thenZero, int durationSec) {
        if (!sensorRunning) {
            synchronized (calibLock) {
                calibStatus = "error_sensor_stopped";
                calibActive = false;
            }
            return;
        }
        int d = Math.max(5, Math.min(60, durationSec));
        synchronized (calibLock) {
            calibSamples.clear();
            calibActive = true;
            calibPendingZero = thenZero;
            calibStartMs = System.currentTimeMillis();
            calibTargetMs = d * 1000L;
            calibStatus = "sampling";
        }
        Log.i(TAG, "Calib start: duration=" + d + "s, thenZero=" + thenZero);
    }

    public synchronized void applyCalibNow() {
        synchronized (calibLock) {
            if (calibSamples.size() < 5) {
                calibStatus = "too_few_samples";
                calibActive = false;
                return;
            }
            applyCalibFromSamples();
        }
    }

    public synchronized void clearCalibStatic() {
        synchronized (calibLock) {
            calibSamples.clear();
            calibActive = false;
            calibPendingZero = false;
            calibStatus = "idle";
            config.posBiasX      = 0.0;
            config.posBiasY      = 0.0;
            config.posDeadbandPx = 0.15;
            config.posCalibrated = false;
            try { ConfigStore.save(getApplicationContext(), config); }
            catch (Exception e) { Log.w(TAG, "save calib clear", e); }
        }
    }

    private void collectCalibSample(double dx, double dy) {
        synchronized (calibLock) {
            if (!calibActive) return;
            if (Math.hypot(dx, dy) > CALIB_REJECT_PX) {
                calibSamples.clear();
                calibActive = false;
                calibPendingZero = false;
                calibStatus = "aborted_motion";
                return;
            }
            calibSamples.add(new double[]{dx, dy});
            long elapsed   = System.currentTimeMillis() - calibStartMs;
            boolean timeUp = elapsed >= calibTargetMs;
            boolean enough = calibSamples.size() >= CALIB_MIN_SAMPLES;
            if (timeUp && enough) { applyCalibFromSamples(); return; }
            if (elapsed >= calibTargetMs * CALIB_MAX_MULTIPLIER) {
                if (calibSamples.size() >= 10) applyCalibFromSamples();
                else { calibStatus = "error_too_few"; calibActive = false; calibPendingZero = false; }
            }
        }
    }

    private void applyCalibFromSamples() {
        int n = calibSamples.size();
        if (n < 10) {
            calibStatus = "error_too_few";
            calibActive = false;
            calibPendingZero = false;
            return;
        }
        double sumX = 0, sumY = 0;
        for (double[] s : calibSamples) { sumX += s[0]; sumY += s[1]; }
        double meanX = sumX / n;
        double meanY = sumY / n;
        double varX = 0, varY = 0;
        for (double[] s : calibSamples) {
            varX += (s[0] - meanX) * (s[0] - meanX);
            varY += (s[1] - meanY) * (s[1] - meanY);
        }
        double stdX = Math.sqrt(varX / Math.max(1, n - 1));
        double stdY = Math.sqrt(varY / Math.max(1, n - 1));
        double std  = Math.max(stdX, stdY);

        config.posBiasX      = meanX;
        config.posBiasY      = meanY;
        config.posDeadbandPx = Math.max(0.15, Math.min(3.0, 3.5 * std));
        config.posCalibrated = true;

        try { ConfigStore.save(getApplicationContext(), config); }
        catch (Exception e) { Log.w(TAG, "save calib", e); }

        boolean setZeroAfter = calibPendingZero;
        calibPendingZero = false;
        calibSamples.clear();
        calibActive = false;
        calibStatus = "done";
        if (setZeroAfter) setZero();
    }

    public JSONObject calibStatusJson() {
        try {
            JSONObject o = new JSONObject();
            synchronized (calibLock) {
                o.put("status",       calibStatus);
                o.put("active",       calibActive);
                o.put("samples",      calibSamples.size());
                o.put("minSamples",   CALIB_MIN_SAMPLES);
                o.put("pendingZero",  calibPendingZero);
                long elapsed = calibActive ? (System.currentTimeMillis() - calibStartMs) : 0;
                o.put("elapsedMs", elapsed);
                o.put("targetMs",  calibTargetMs);
                double progress;
                if (calibActive && calibTargetMs > 0) progress = Math.min(1.0, (double) elapsed / calibTargetMs);
                else if ("done".equals(calibStatus)) progress = 1.0;
                else progress = 0.0;
                o.put("progress", progress);
                double remainSec = 0;
                if (calibActive && calibTargetMs > 0)
                    remainSec = Math.max(0, (calibTargetMs - elapsed) / 1000.0);
                o.put("remainSec", remainSec);
            }
            o.put("posBiasX",        config.posBiasX);
            o.put("posBiasY",        config.posBiasY);
            o.put("posDeadbandPx",   config.posDeadbandPx);
            o.put("posCalibrated",   config.posCalibrated);
            o.put("durationSec",     config.calibDurationSec);
            return o;
        } catch (Exception e) { return new JSONObject(); }
    }

    public SnapshotProvider getSnapshotProvider() { return snapshotProvider; }

    public synchronized void patchConfig(JSONObject patch) {
        int oldCameraId = config.cameraId;
        int oldPreviewW = config.cameraPreviewWidth;
        int oldPreviewH = config.cameraPreviewHeight;

        config.applyPatch(patch);
        try { ConfigStore.save(getApplicationContext(), config); }
        catch (Exception e) { Log.w(TAG, "config save", e); }

        if (flashlight != null && patch.has("flashlightMode"))
            flashlight.setMode(config.flashlightMode);
        if (flashlight != null && patch.has("lightThreshold"))
            flashlight.setThreshold(config.lightThreshold);
        if (patch.has("clientEnabled") || patch.has("clientProtocol")
                || patch.has("clientServerUrl") || patch.has("clientPath")
                || patch.has("clientToken") || patch.has("clientIntervalMs"))
            restartClient();

        boolean camChanged = (config.cameraId != oldCameraId);
        boolean sizeChanged = (config.cameraPreviewWidth != oldPreviewW
                || config.cameraPreviewHeight != oldPreviewH);
        if ((camChanged || sizeChanged) && sensorRunning) {
            stopSensorOnly();
            startSensorOnly();
        }
    }

    // ==============================================================
    // HTTP port change
    // ==============================================================

    public synchronized boolean scheduleHttpRestart(final int newPort) {
        if (newPort <= 0 || newPort > 65535) return false;
        final int oldPort = config.httpPort;
        if (newPort == oldPort && httpServer != null) return true;
        config.httpPort = newPort;
        try { ConfigStore.save(getApplicationContext(), config); }
        catch (Exception e) { Log.w(TAG, "config save (httpPort)", e); }

        Thread t = new Thread(new Runnable() {
            @Override public void run() {
                try { Thread.sleep(300); } catch (InterruptedException ignored) {}
                doHttpRestart(oldPort, newPort);
            }
        }, "HttpRestart");
        t.setDaemon(true);
        t.start();
        return true;
    }

    private synchronized void doHttpRestart(int oldPort, int newPort) {
        if (httpServer != null) {
            try { httpServer.stop(); } catch (Exception ignored) {}
            httpServer = null;
        }
        int actualPort = oldPort;
        if (config.restEnabled || config.sseEnabled) {
            HttpApiServer srv = new HttpApiServer(getApplicationContext(), this);
            try {
                srv.start(newPort, config.httpHost);
                httpServer = srv;
                sCurrentHttpPort = newPort;
                actualPort = newPort;
            } catch (Exception e) {
                Log.e(TAG, "Failed to start HTTP on " + newPort, e);
                if (newPort != oldPort) {
                    config.httpPort = oldPort;
                    try { ConfigStore.save(getApplicationContext(), config); } catch (Exception ignored) {}
                    HttpApiServer srvOld = new HttpApiServer(getApplicationContext(), this);
                    try {
                        srvOld.start(oldPort, config.httpHost);
                        httpServer = srvOld;
                        sCurrentHttpPort = oldPort;
                        actualPort = oldPort;
                    } catch (Exception e2) { Log.e(TAG, "Rollback failed", e2); }
                }
            }
        } else {
            sCurrentHttpPort = newPort;
            actualPort = newPort;
        }
        notifyPortChanged(newPort, actualPort);
        updateNotification(buildStatusLine());
    }

    // ==============================================================
    // Servers
    // ==============================================================

    private void startServers() {
        if (config.restEnabled || config.sseEnabled) {
            httpServer = new HttpApiServer(getApplicationContext(), this);
            try {
                httpServer.start(config.httpPort, config.httpHost);
                sCurrentHttpPort = config.httpPort;
            } catch (Exception e) { Log.e(TAG, "HTTP start", e); }
        }
        if (config.wsEnabled) {
            wsServer = new WebSocketServer(this);
            try { wsServer.start(config.wsPort); } catch (Exception e) { Log.e(TAG, "WS start", e); }
        }
        if (config.telnetEnabled) {
            telnetServer = new TelnetServer(this);
            try { telnetServer.start(config.telnetPort); } catch (Exception e) { Log.e(TAG, "Telnet start", e); }
        }
    }

    private void stopServers() {
        if (httpServer != null)   { httpServer.stop();   httpServer = null; }
        if (wsServer != null)     { wsServer.stop();     wsServer = null; }
        if (telnetServer != null) { telnetServer.stop(); telnetServer = null; }
    }

    private void startClient() {
        pushClient = new PushClient(broadcaster);
        pushClient.start(config);
    }

    private void restartClient() {
        if (pushClient != null) { pushClient.stop(); pushClient = null; }
        if (config.clientEnabled) startClient();
    }

    // ==============================================================
    // Camera
    // ==============================================================

    public synchronized void startSensorOnly() {
        if (sensorRunning) return;

        // --- ПРОВЕРКА РАЗРЕШЕНИЙ (Android 6+) ---
        if (Build.VERSION.SDK_INT >= 23) {
            int check = checkSelfPermission(android.Manifest.permission.CAMERA);
            if (check != android.content.pm.PackageManager.PERMISSION_GRANTED) {
                lastCameraError = "Нет разрешения на камеру. Откройте настройки приложения и разрешите доступ к камере.";
                Log.e(TAG, lastCameraError);
                updateNotification(buildStatusLine());
                return;
            }
        }

        tracker.configure(
                config.targetWidth, config.searchRadius, config.blockSize,
                config.flowMinVariance, config.flowMinGradient, config.maxSadPx,
                config.flowSubpixel, config.flowTemporalSmoothing,
                config.flowTemporalAlpha, config.flowMaxMotionPx,
                config.trackerContrast);

        if (worker == null) {
            worker = new HandlerThread("SensorWorker");
            worker.start();
            workerHandler = new Handler(worker.getLooper());
        }

        if (!openCamera()) {
            // lastCameraError уже установлен внутри openCamera()
            updateNotification(buildStatusLine());
            return;
        }

        try {
            flashlight = new FlashlightController(getApplicationContext());
            flashlight.init();
            flashlight.setThreshold(config.lightThreshold);
            flashlight.setMode(config.flashlightMode);
            flashlight.setCamera(camera);
        } catch (Exception e) {
            flashlight = null;
        }

        sensorRunning = true;
        lastCameraError = "";
        lastPublishMs   = 0L;
        lastFlashState  = "";
        skippedReadings = 0L;

        synchronized (positionLock) {
            posX = 0.0; posY = 0.0;
            zeroX = 0.0; zeroY = 0.0;
            worldX = 0.0; worldY = 0.0;
            zeroWorldX = 0.0; zeroWorldY = 0.0;
            rotZeroPx = 0.0;
            zeroSetAtMs = System.currentTimeMillis();
        }
        totalRotationDeg = 0.0;
        rotCalibActive = false;
        synchronized (trailLock) {
            trail.clear();
            trail.add(new double[]{0, 0, 0, System.currentTimeMillis()});
            lastTrailX = 0;
            lastTrailY = 0;
        }
        settleUntilMs = System.currentTimeMillis() + 300;
        forcePublish  = true;

        synchronized (calibLock) {
            calibSamples.clear();
            calibActive = false;
            calibPendingZero = false;
            calibStatus = config.posCalibrated ? "done" : "idle";
        }

        Log.i(TAG, "Sensor started " + previewWidth + "x" + previewHeight);
        updateNotification(buildStatusLine());
    }

    public synchronized void stopSensorOnly() {
        sensorRunning = false;
        synchronized (positionLock) {
            posX = 0.0; posY = 0.0;
            zeroX = 0.0; zeroY = 0.0;
            worldX = 0.0; worldY = 0.0;
            zeroWorldX = 0.0; zeroWorldY = 0.0;
            rotZeroPx = 0.0;
            zeroSetAtMs = System.currentTimeMillis();
        }
        rotCalibActive = false;
        if (flashlight != null) {
            try { flashlight.release(); } catch (Exception ignored) {}
            flashlight = null;
        }

        synchronized (snapshotLock) { fullFrame = null; }

        if (camera != null) {
            try { camera.setPreviewCallbackWithBuffer(null); } catch (Exception ignored) {}
            try { camera.stopPreview(); } catch (Exception ignored) {}
            try { camera.release(); } catch (Exception ignored) {}
            camera = null;
        }
        if (dummyTexture != null) {
            try { dummyTexture.release(); } catch (Exception ignored) {}
            dummyTexture = null;
        }
        if (worker != null) { worker.quit(); worker = null; workerHandler = null; }

        updateNotification(buildStatusLine());
    }

    private boolean openCamera() {
        if (camera != null) {
            try { camera.setPreviewCallbackWithBuffer(null); } catch (Exception ignored) {}
            try { camera.stopPreview(); } catch (Exception ignored) {}
            try { camera.release(); } catch (Exception ignored) {}
            camera = null;
        }
        if (dummyTexture != null) {
            try { dummyTexture.release(); } catch (Exception ignored) {}
            dummyTexture = null;
        }

        int totalCameras = 0;
        try { totalCameras = android.hardware.Camera.getNumberOfCameras(); } catch (Throwable t) { Log.e(TAG, "getNumberOfCameras failed", t); }
        Log.i(TAG, "Total cameras: " + totalCameras);

        if (totalCameras == 0) {
            lastCameraError = "В системе не найдено ни одной камеры (getNumberOfCameras=0).";
            return false;
        }

        List<Integer> idsToTry = new ArrayList<>();
        if (config.cameraId >= 0 && config.cameraId < totalCameras) {
            idsToTry.add(config.cameraId);
        }
        for (int i = 0; i < totalCameras; i++) {
            if (!idsToTry.contains(i)) idsToTry.add(i);
        }

        String lastErr = "";
        for (int cameraId : idsToTry) {
            for (int attempt = 1; attempt <= 2; attempt++) {
                try {
                    Log.i(TAG, "Trying Camera.open(" + cameraId + "), attempt " + attempt);
                    camera = android.hardware.Camera.open(cameraId);
                    if (camera == null) {
                        lastErr = "Camera.open(" + cameraId + ") вернул null";
                        Log.w(TAG, lastErr);
                        sleepMs(400);
                        continue;
                    }
                    if (!configureCamera()) {
                        lastErr = "configureCamera() не смог настроить камеру " + cameraId;
                        Log.w(TAG, lastErr);
                        try { camera.release(); } catch (Exception ignored) {}
                        camera = null;
                        sleepMs(400);
                        continue;
                    }
                    try {
                        dummyTexture = new SurfaceTexture(0);
                        // ВАЖНО: Некоторые драйверы (особенно USB-камеры) требуют явно задать размер буфера,
                        // иначе startPreview завершается с ошибкой.
                        dummyTexture.setDefaultBufferSize(previewWidth, previewHeight);
                        camera.setPreviewTexture(dummyTexture);
                    } catch (Exception e) {
                        lastErr = "setPreviewTexture failed: " + e.getMessage();
                        Log.e(TAG, lastErr, e);
                        try { camera.release(); } catch (Exception ignored) {}
                        camera = null;
                        if (dummyTexture != null) { try { dummyTexture.release(); } catch (Exception ignored) {} dummyTexture = null; }
                        sleepMs(400);
                        continue;
                    }
                    try {
                        camera.startPreview();
                    } catch (Exception e) {
                        lastErr = "startPreview: " + e.getMessage();
                        Log.e(TAG, lastErr, e);
                        try { camera.release(); } catch (Exception ignored) {}
                        camera = null;
                        if (dummyTexture != null) { try { dummyTexture.release(); } catch (Exception ignored) {} dummyTexture = null; }
                        sleepMs(400);
                        continue;
                    }
                    Log.i(TAG, "Camera " + cameraId + " opened OK. " + previewWidth + "x" + previewHeight);
                    config.cameraId = cameraId;
                    return true;
                } catch (Throwable e) {
                    lastErr = e.getClass().getSimpleName() + ": " + e.getMessage();
                    Log.e(TAG, "openCamera(" + cameraId + ") failed", e);
                    if (camera != null) { try { camera.release(); } catch (Exception ignored) {} camera = null; }
                    if (dummyTexture != null) { try { dummyTexture.release(); } catch (Exception ignored) {} dummyTexture = null; }
                    sleepMs(400);
                }
            }
        }

        lastCameraError = "Не удалось открыть камеру. Причина: " + lastErr
                + ". Проверьте, что камера не занята другим приложением и разрешение выдано.";
        return false;
    }

    private boolean configureCamera() {
        try {
            Camera.Parameters p = camera.getParameters();
            Log.i(TAG, "Supported preview sizes: " + (p.getSupportedPreviewSizes() != null ? p.getSupportedPreviewSizes().size() : 0));

            if (cachedCameraSizes == null) {
                List<Camera.Size> sizes = p.getSupportedPreviewSizes();
                if (sizes != null) {
                    java.util.ArrayList<Camera.Size> sorted = new java.util.ArrayList<Camera.Size>(sizes);
                    Collections.sort(sorted, new Comparator<Camera.Size>() {
                        @Override public int compare(Camera.Size a, Camera.Size b) {
                            return a.width * a.height - b.width * b.height;
                        }
                    });
                    org.json.JSONArray arr = new org.json.JSONArray();
                    for (Camera.Size s : sorted) {
                        try {
                            JSONObject o = new JSONObject();
                            o.put("width", s.width);
                            o.put("height", s.height);
                            o.put("area", s.width * s.height);
                            arr.put(o);
                        } catch (Exception ignored) {}
                    }
                    cachedCameraSizes = arr;
                }
            }

            Camera.Size size = pickPreviewSize(p.getSupportedPreviewSizes());
            if (size == null) {
                Log.e(TAG, "pickPreviewSize returned null");
                return false;
            }
            previewWidth = size.width;
            previewHeight = size.height;
            Log.i(TAG, "Selected preview size: " + previewWidth + "x" + previewHeight);

            p.setPreviewSize(previewWidth, previewHeight);

            // Некоторые старые камеры не поддерживают NV21 — проверяем и пробуем YV12
            int previewFormat = ImageFormat.NV21;
            try {
                p.setPreviewFormat(ImageFormat.NV21);
            } catch (Exception e) {
                Log.w(TAG, "NV21 not supported, trying YV12: " + e.getMessage());
                previewFormat = ImageFormat.YV12;
                p.setPreviewFormat(previewFormat);
            }

            try {
                List<int[]> ranges = p.getSupportedPreviewFpsRange();
                if (ranges != null && !ranges.isEmpty()) {
                    int[] best = ranges.get(0);
                    for (int[] r : ranges) {
                        if (r[1] > best[1] || (r[1] == best[1] && r[0] < best[0])) best = r;
                    }
                    p.setPreviewFpsRange(best[0], best[1]);
                    Log.i(TAG, "FPS range: " + best[0] + "-" + best[1]);
                }
            } catch (Exception ignore) {}

            try {
                List<String> focus = p.getSupportedFocusModes();
                if (focus != null) {
                    if (focus.contains(Camera.Parameters.FOCUS_MODE_INFINITY))
                        p.setFocusMode(Camera.Parameters.FOCUS_MODE_INFINITY);
                    else if (focus.contains(Camera.Parameters.FOCUS_MODE_FIXED))
                        p.setFocusMode(Camera.Parameters.FOCUS_MODE_FIXED);
                }
            } catch (Exception ignore) {}

            try {
                if (p.isVideoStabilizationSupported()) p.setVideoStabilization(false);
            } catch (Exception ignore) {}

            camera.setParameters(p);
            Log.i(TAG, "Camera parameters applied OK");

            int bufSize = previewWidth * previewHeight
                    * ImageFormat.getBitsPerPixel(previewFormat) / 8;
            yPlane = new byte[previewWidth * previewHeight];
            tracker.reset(previewWidth, previewHeight);

            camera.setPreviewCallbackWithBuffer(new Camera.PreviewCallback() {
                @Override public void onPreviewFrame(byte[] data, Camera cam) {
                    onFrame(data, cam);
                }
            });
            camera.addCallbackBuffer(new byte[bufSize]);
            camera.addCallbackBuffer(new byte[bufSize]);
            return true;
        } catch (Exception e) {
            Log.e(TAG, "configureCamera exception", e);
            return false;
        }
    }

    public org.json.JSONArray getCameraSizes() {
        return cachedCameraSizes != null ? cachedCameraSizes : new org.json.JSONArray();
    }
    private static void sleepMs(long ms) { try { Thread.sleep(ms); } catch (InterruptedException ignored) {} }

    private Camera.Size pickPreviewSize(List<Camera.Size> sizes) {
        if (sizes == null || sizes.isEmpty()) return null;

        // Если в конфиге задан конкретный размер — пробуем его
        if (config.cameraPreviewWidth > 0 && config.cameraPreviewHeight > 0) {
            for (Camera.Size s : sizes) {
                if (s.width == config.cameraPreviewWidth && s.height == config.cameraPreviewHeight)
                    return s;
            }
            Log.w(TAG, "Configured size " + config.cameraPreviewWidth + "x"
                    + config.cameraPreviewHeight + " not in supported list — using auto");
        }

        // 1. Ищем стандартный 640x480 (VGA) — наиболее совместим с USB-камерами
        for (Camera.Size s : sizes) {
            if (s.width == 640 && s.height == 480) return s;
        }
        // 2. Ищем 320x240 (QVGA)
        for (Camera.Size s : sizes) {
            if (s.width == 320 && s.height == 240) return s;
        }
        // 3. Любой размер >= 320x240 с минимальной площадью
        Camera.Size best = null;
        for (Camera.Size s : sizes) {
            if (s.width >= 320 && s.height >= 240) {
                if (best == null || s.width * s.height < best.width * best.height) best = s;
            }
        }
        if (best != null) return best;

        // 4. Fallback на минимальный >= 128x96
        for (Camera.Size s : sizes) {
            if (s.width >= 128 && s.height >= 96) {
                if (best == null || s.width * s.height < best.width * best.height) best = s;
            }
        }
        return best != null ? best : sizes.get(0);
    }

    // ==============================================================
    // Frame processing
    // ==============================================================

    private void onFrame(byte[] data, Camera cam) {
        if (cam == null) return;
        if (data == null || yPlane == null) {
            if (data != null) cam.addCallbackBuffer(data);
            return;
        }
        if (!frameBusy.compareAndSet(false, true)) {
            cam.addCallbackBuffer(data);
            return;
        }
        System.arraycopy(data, 0, yPlane, 0, yPlane.length);
        synchronized (snapshotLock) {
            if (fullFrame == null || fullFrame.length != data.length)
                fullFrame = new byte[data.length];
            System.arraycopy(data, 0, fullFrame, 0, data.length);
        }
        cam.addCallbackBuffer(data);

        final long ts = System.currentTimeMillis();
        Handler h = workerHandler;
        if (h != null) h.post(new Runnable() { @Override public void run() { processFrame(ts); } });
        else frameBusy.set(false);
    }

    public byte[] encodeSnapshotJpeg(int quality) {
        byte[] frameCopy;
        int w, h;
        synchronized (snapshotLock) {
            if (fullFrame == null || previewWidth <= 0 || previewHeight <= 0) return null;
            w = previewWidth; h = previewHeight;
            frameCopy = new byte[fullFrame.length];
            System.arraycopy(fullFrame, 0, frameCopy, 0, fullFrame.length);
        }
        int ySize = w * h;
        if (config.grayscaleOutput) {
            for (int i = ySize; i < frameCopy.length; i++) frameCopy[i] = (byte) 128;
        }
        if (config.outputContrast != 100) {
            int c = config.outputContrast;
            for (int i = 0; i < ySize; i++) {
                int v = frameCopy[i] & 0xFF;
                int v2 = 128 + ((v - 128) * c) / 100;
                if (v2 < 0) v2 = 0; else if (v2 > 255) v2 = 255;
                frameCopy[i] = (byte) v2;
            }
        }
        int rotation = config.cameraPreviewRotation;
        rotation = ((rotation % 360) + 360) % 360;
        if (rotation != 90 && rotation != 180 && rotation != 270) rotation = 0;
        try {
            android.graphics.YuvImage yuv = new android.graphics.YuvImage(
                    frameCopy, ImageFormat.NV21, w, h, null);
            java.io.ByteArrayOutputStream baos = new java.io.ByteArrayOutputStream();
            yuv.compressToJpeg(new android.graphics.Rect(0, 0, w, h), quality, baos);
            byte[] jpeg0 = baos.toByteArray();
            if (rotation == 0) return jpeg0;
            android.graphics.Bitmap src = android.graphics.BitmapFactory.decodeByteArray(
                    jpeg0, 0, jpeg0.length, null);
            if (src == null) return jpeg0;
            android.graphics.Matrix m = new android.graphics.Matrix();
            m.postRotate(rotation);
            android.graphics.Bitmap rotated = android.graphics.Bitmap.createBitmap(
                    src, 0, 0, src.getWidth(), src.getHeight(), m, true);
            if (rotated != src) { try { src.recycle(); } catch (Exception ignored) {} }
            java.io.ByteArrayOutputStream out = new java.io.ByteArrayOutputStream();
            rotated.compress(android.graphics.Bitmap.CompressFormat.JPEG, quality, out);
            try { rotated.recycle(); } catch (Exception ignored) {}
            return out.toByteArray();
        } catch (Throwable t) {
            Log.e(TAG, "encodeSnapshotJpeg", t);
            return null;
        }
    }

    public byte[] encodeDebugJpeg() {
        OpticalFlowTracker.Result r;
        synchronized (flowResultLock) { r = lastFlowResult; }
        int gw = tracker.getGridWidth();
        int gh = tracker.getGridHeight();
        if (gw <= 0 || gh <= 0) return null;
        byte[] prev = tracker.getPrevBuffer();
        if (prev == null || prev.length != gw * gh) return null;

        int[] pixels = new int[gw * gh];
        for (int i = 0; i < gw * gh; i++) {
            int v = prev[i] & 0xFF;
            pixels[i] = 0xFF000000 | (v << 16) | (v << 8) | v;
        }

        // Рисуем крестики на найденных фичах optical flow
        if (r != null && r.featureXs != null) {
            for (int i = 0; i < r.featureCount; i++) {
                int fx = r.featureXs[i], fy = r.featureYs[i];
                for (int d = -2; d <= 2; d++) {
                    int x = fx + d, y = fy;
                    if (x >= 0 && x < gw && y >= 0 && y < gh) pixels[y * gw + x] = 0xFFFF4040;
                    int x2 = fx, y2 = fy + d;
                    if (x2 >= 0 && x2 < gw && y2 >= 0 && y2 < gh) pixels[y2 * gw + x2] = 0xFFFF4040;
                }
            }
        }

        try {
            android.graphics.Bitmap bmp = android.graphics.Bitmap.createBitmap(
                    gw, gh, android.graphics.Bitmap.Config.ARGB_8888);
            bmp.setPixels(pixels, 0, gw, 0, 0, gw, gh);
            int scale = 4;
            android.graphics.Bitmap scaled = android.graphics.Bitmap.createScaledBitmap(
                    bmp, gw * scale, gh * scale, false);
            bmp.recycle();

            // ============================================================
            // РИСОВАНИЕ ВЕКТОРА НАПРАВЛЕНИЯ ДВИЖЕНИЯ
            // ============================================================
            android.graphics.Canvas canvas = new android.graphics.Canvas(scaled);
            android.graphics.Paint arrowPaint = new android.graphics.Paint();
            arrowPaint.setColor(0xFF00FF00); // Ярко-зеленый
            arrowPaint.setStyle(android.graphics.Paint.Style.STROKE);
            arrowPaint.setStrokeWidth(4);
            arrowPaint.setAntiAlias(true);

            android.graphics.Paint centerPaint = new android.graphics.Paint();
            centerPaint.setColor(0xFF00FFFF); // Голубой
            centerPaint.setStyle(android.graphics.Paint.Style.FILL);
            centerPaint.setAntiAlias(true);

            float cx = scaled.getWidth() / 2f;
            float cy = scaled.getHeight() / 2f;

            // Рисуем вектор, только если движение валидно и есть смещение
            if (r != null && r.valid && r.distancePx > 0.05) {
                double drawDx = r.dxPx;
                double drawDy = r.dyPx;

                // Учитываем настройки монтажа и инверсии
                int rot = ((config.cameraMountRotationDeg % 360) + 360) % 360;
                if (rot == 90)       { double t = drawDx; drawDx = -drawDy; drawDy = t; }
                else if (rot == 180) { drawDx = -drawDx; drawDy = -drawDy; }
                else if (rot == 270) { double t = drawDx; drawDx = drawDy; drawDy = -t; }

                if (config.invertDx) drawDx = -drawDx;
                if (config.invertDy) drawDy = -drawDy;

                // Масштабируем вектор для наглядности на экране
                float visualScale = (float) (4.0 / tracker.getDownscale());
                float factor = visualScale * 5.0f;

                // КОРРЕКЦИЯ ДЛЯ ПОВОРОТА КАРТИНКИ НА 90 ГРАДУСОВ ВПРАВО:
                // Так как мы повернем картинку на 90 градусов по часовой стрелке в конце,
                // нам нужно нарисовать стрелку так, чтобы после поворота она смотрела правильно.
                // Физическое "Вперед" (положительный drawDy) должно смотреть "Вверх" после поворота.
                // В системе координат Canvas (до поворота) "Вверх" после поворота на 90 CW — это "Влево" (отрицательный X).
                // Значит, для drawDy мы используем -X. Для drawDx (Вправо) мы используем -Y (Вверх).
                float arrowVecX = (float)(-drawDy);
                float arrowVecY = (float)(-drawDx);

                float endX = cx + arrowVecX * factor;
                float endY = cy + arrowVecY * factor;

                float dist = (float)Math.hypot(endX - cx, endY - cy);
                if (dist > 10) { // Рисуем, если стрелка достаточно длинная
                    // Линия от центра до конца вектора
                    canvas.drawLine(cx, cy, endX, endY, arrowPaint);

                    // Наконечник стрелки
                    float arrowAngle = (float)Math.atan2(endY - cy, endX - cx);
                    float arrowSize = 20f;
                    float a1 = arrowAngle + (float)Math.PI * 0.85f;
                    float a2 = arrowAngle - (float)Math.PI * 0.85f;

                    float hx1 = endX + (float)Math.cos(a1) * arrowSize;
                    float hy1 = endY + (float)Math.sin(a1) * arrowSize;
                    float hx2 = endX + (float)Math.cos(a2) * arrowSize;
                    float hy2 = endY + (float)Math.sin(a2) * arrowSize;

                    canvas.drawLine(endX, endY, hx1, hy1, arrowPaint);
                    canvas.drawLine(endX, endY, hx2, hy2, arrowPaint);
                }
            }
            // Точка в центре координат (откуда растет вектор)
            canvas.drawCircle(cx, cy, 5, centerPaint);
            // ============================================================

            // ============================================================
            // ПОВОРОТ ИТОГОВОЙ КАРТИНКИ (вместе со стрелкой) НА 90 ГРАДУСОВ
            // ============================================================
            android.graphics.Matrix m = new android.graphics.Matrix();
            m.postRotate(90);
            android.graphics.Bitmap rotated = android.graphics.Bitmap.createBitmap(
                    scaled, 0, 0, scaled.getWidth(), scaled.getHeight(), m, true);
            if (rotated != scaled) {
                try { scaled.recycle(); } catch (Exception ignored) {}
            }

            java.io.ByteArrayOutputStream out = new java.io.ByteArrayOutputStream();
            // Используем JPEG для потоковой передачи
            rotated.compress(android.graphics.Bitmap.CompressFormat.JPEG, 85, out);
            rotated.recycle();
            return out.toByteArray();

        } catch (Throwable t) {
            Log.e(TAG, "encodeDebugJpeg", t);
            return null;
        }
    }

    private void processFrame(long timestamp) {
        try {
            OpticalFlowTracker.Result r = tracker.update(yPlane, previewWidth, previewHeight);
            seq++;

            // ---- 1. Оси по настройкам монтажа ----
            double dx = r.dxPx, dy = r.dyPx;
            int rot = ((config.cameraMountRotationDeg % 360) + 360) % 360;
            if (rot == 90)       { double t = dx; dx = -dy; dy = t; }
            else if (rot == 180) { dx = -dx; dy = -dy; }
            else if (rot == 270) { double t = dx; dx = dy; dy = -t; }
            if (config.invertDx) dx = -dx;
            if (config.invertDy) dy = -dy;

            // ---- 2. Settle / force ----
            boolean settling = timestamp < settleUntilMs;
            boolean forced   = forcePublish;
            if (forced) forcePublish = false;
            boolean skipAccum = settling || forced;

            // ---- 3. Накопление с МЯГКИМ deadband, плюс пересчёт мировой позиции ----
            if (r.valid && !skipAccum) {
                collectCalibSample(dx, dy);

                double adjDx = dx - config.posBiasX;
                double adjDy = dy - config.posBiasY;

                double threshold = config.posDeadbandPx;
                double ddx = 0.0, ddy = 0.0;
                if (Math.abs(adjDx) >= threshold)
                    ddx = adjDx - Math.signum(adjDx) * threshold;
                if (Math.abs(adjDy) >= threshold)
                    ddy = adjDy - Math.signum(adjDy) * threshold;

                if (ddx != 0.0 || ddy != 0.0) {
                    synchronized (positionLock) {
                        // Новая интерпретация:
                        //   Y-поток (ddy) → вперёд/назад (posX)
                        //   X-поток (ddx) → поворот (posY, знак как в старом коде)
                        posX += ddy;      // forward accumulator
                        posY += -ddx;     // rotation accumulator

                        // Пересчёт мировой позиции из forward + текущий угол
                        double rotDeg = computeRotationDeg();
                        double rotRad = Math.toRadians(rotDeg);
                        double deltaForward = ddy;  // Δ posX за этот кадр
                        worldX += deltaForward * Math.sin(rotRad);
                        worldY += deltaForward * Math.cos(rotRad);
                    }
                }


                // Трейл (мировые координаты)
                synchronized (trailLock) {
                    double dxT = worldX - lastTrailX;
                    double dyT = worldY - lastTrailY;
                    if (trail.isEmpty() || Math.hypot(dxT, dyT) >= TRAIL_MIN_DIST) {
                        trail.add(new double[]{worldX, worldY, computeRotationDeg(), timestamp});
                        lastTrailX = worldX;
                        lastTrailY = worldY;
                        while (trail.size() > MAX_TRAIL_POINTS) trail.remove(0);
                    }
                }
            }

            // ---- 4. JSON ----
            double distance     = Math.hypot(dx, dy);
            double directionDeg = normalizeDeg(Math.toDegrees(Math.atan2(dy, dx)));
            double rotNow       = computeRotationDeg();

            JSONObject reading = new JSONObject();
            reading.put("seq", seq);
            reading.put("timestamp_ms", timestamp);
            reading.put("datetime", iso.format(new Date(timestamp)));
            long dt = (lastTimestamp > 0 && timestamp >= lastTimestamp)
                    ? (timestamp - lastTimestamp) : 0;
            reading.put("dt_ms", dt);
            lastTimestamp = timestamp;

            JSONObject frame = new JSONObject();
            frame.put("width", previewWidth);
            frame.put("height", previewHeight);
            frame.put("downscale", tracker.getDownscale());
            reading.put("frame", frame);

            JSONObject shift = new JSONObject();
            shift.put("valid",               r.valid);
            shift.put("dx_px",               round(dx, 3));
            shift.put("dy_px",               round(dy, 3));
            shift.put("distance_px",         round(distance, 3));
            shift.put("direction_deg",       round(directionDeg, 2));
            shift.put("rotation_deg",        round(r.rotationDeg, 3));  // per-frame
            shift.put("total_rotation_deg",  round(normalizeDeg(rotNow), 2));
            shift.put("mount_rotation_deg",  rot);
            shift.put("invert_dx",           config.invertDx);
            shift.put("invert_dy",           config.invertDy);
            shift.put("settling",            settling);
            shift.put("skip_accum",          skipAccum);
            reading.put("shift", shift);

            reading.put("position", positionJson());

            JSONObject q = new JSONObject();
            q.put("blocks_used",           r.blocksUsed);
            q.put("blocks_total",          r.blocksTotal);
            q.put("blocks_passed_quality", r.blocksPassedQuality);
            q.put("confidence",            round(r.confidence, 3));
            q.put("feature_count",         r.featureCount);
            reading.put("quality", q);

            if (flashlight != null) {
                JSONObject f = new JSONObject();
                f.put("mode",      flashlight.getMode());
                f.put("on",        flashlight.isOn());
                f.put("available", flashlight.isAvailable());
                f.put("lux",       flashlight.getLux());
                reading.put("flashlight", f);
            }
            reading.put("proc_ms", round(r.processNs / 1e6, 2));

            synchronized (flowResultLock) { lastFlowResult = r; }

            // ---- 5. Публикация ----
            boolean shouldPublish;
            String  reason;

            if (forced) {
                shouldPublish = true;  reason = settling ? "force_settling" : "force";
            } else if (!config.publishOnlyOnChange) {
                shouldPublish = true;  reason = "always";
            } else if (lastPublishMs == 0L) {
                shouldPublish = true;  reason = "first";
            } else if (!r.valid) {
                shouldPublish = false; reason = "invalid";
            } else if (r.confidence < 0.10) {
                shouldPublish = false; reason = "low_confidence";
            } else {
                boolean moved = Math.abs(dx) >= config.publishMotionThresholdPx
                        || Math.abs(dy) >= config.publishMotionThresholdPx
                        || Math.abs(r.rotationDeg) >= config.publishRotationThresholdDeg;
                String newFlashState = buildFlashState();
                boolean flashChanged = !newFlashState.equals(lastFlashState);
                boolean heartbeat = config.publishHeartbeatMs > 0
                        && (timestamp - lastPublishMs) >= config.publishHeartbeatMs;

                if (moved)              { shouldPublish = true;  reason = "motion"; }
                else if (flashChanged)  { shouldPublish = true;  reason = "flashlight"; }
                else if (heartbeat)     { shouldPublish = true;  reason = "heartbeat"; }
                else                    { shouldPublish = false; reason = "unchanged"; }
            }

            reading.put("publish_reason", reason);

            if (shouldPublish) {
                broadcaster.publish(reading);
                lastPublishMs  = timestamp;
                lastFlashState = buildFlashState();
            } else {
                broadcaster.setLatest(reading);
                skippedReadings++;
            }
        } catch (Throwable t) {
            Log.e(TAG, "processFrame", t);
        } finally {
            frameBusy.set(false);
        }
    }

    private static double round(double v, int d) {
        if (Double.isNaN(v) || Double.isInfinite(v)) return 0;
        double p = Math.pow(10, d);
        return Math.round(v * p) / p;
    }

    private static double normalizeDeg(double a) {
        a = a % 360.0;
        if (a < 0) a += 360.0;
        return a;
    }

    private String buildFlashState() {
        if (flashlight == null) return "none";
        return flashlight.getMode()
                + "|" + (flashlight.isOn() ? "1" : "0")
                + "|" + (flashlight.isAvailable() ? "1" : "0");
    }

    // ==============================================================
    // positionJson / mapJson — с новыми полями
    // ==============================================================

    private JSONObject positionJson() {
        try {
            double rx, ry, px, py, zx, zy, wx, wy, zwx, zwy;
            long t;
            synchronized (positionLock) {
                rx = posX - zeroX;
                ry = posY - zeroY;
                px = posX; py = posY;
                zx = zeroX; zy = zeroY;
                wx = worldX - zeroWorldX;
                wy = worldY - zeroWorldY;
                zwx = zeroWorldX; zwy = zeroWorldY;
                t = zeroSetAtMs;
            }
            double rot = computeRotationDeg();

            JSONObject o = new JSONObject();
            // Device-frame (обратная совместимость)
            o.put("pos_raw_x_px", round(px, 3));
            o.put("pos_raw_y_px", round(py, 3));
            o.put("zero_x_px",    round(zx, 3));
            o.put("zero_y_px",    round(zy, 3));
            // World-frame — теперь это «x_px/y_px» в JSON
            o.put("x_px",         round(wx, 3));
            o.put("y_px",         round(wy, 3));
            o.put("distance_from_zero_px",  round(Math.hypot(wx, wy), 3));
            o.put("direction_from_zero_deg",
                    round(normalizeDeg(Math.toDegrees(Math.atan2(wy, wx))), 2));
            o.put("world_x_px",   round(wx, 3));
            o.put("world_y_px",   round(wy, 3));
            o.put("zero_world_x_px", round(zwx, 3));
            o.put("zero_world_y_px", round(zwy, 3));
            // Rotation
            o.put("rotation_deg",         round(normalizeDeg(rot), 2));
            o.put("rotation_raw_deg",     round(rot, 3));
            o.put("rotation_calibrated",  config.rotCalibDone);
            o.put("zero_set_at_ms", t);
            return o;
        } catch (Exception e) {
            return new JSONObject();
        }
    }

    public JSONObject positionJsonPublic() { return positionJson(); }

    public JSONObject mapJson() {
        try {
            JSONObject o = new JSONObject();
            double wx, wy, zeroXc, zeroYc;
            long t0;
            synchronized (positionLock) {
                wx = worldX - zeroWorldX;
                wy = worldY - zeroWorldY;
                zeroXc = zeroWorldX;
                zeroYc = zeroWorldY;
                t0 = zeroSetAtMs;
            }
            double rot = computeRotationDeg();

            o.put("x_px",              round(wx, 3));
            o.put("y_px",              round(wy, 3));
            o.put("rotation_deg",      round(normalizeDeg(rot), 2));
            o.put("rotation_raw_deg",  round(rot, 3));
            o.put("rotation_calibrated", config.rotCalibDone);
            o.put("zero_set_at_ms",    t0);
            o.put("calibrated",        config.posCalibrated);
            o.put("deadband_px",       config.posDeadbandPx);
            o.put("rot_deadband_deg",  config.rotDeadbandDeg);
            o.put("sensor_running",    sensorRunning);
            o.put("rot_calib",         rotationCalibJson());

            org.json.JSONArray arr = new org.json.JSONArray();
            synchronized (trailLock) {
                for (double[] pt : trail) {
                    JSONObject p = new JSONObject();
                    p.put("x",   round(pt[0] - zeroXc, 2));
                    p.put("y",   round(pt[1] - zeroYc, 2));
                    p.put("rot", round(normalizeDeg(pt[2]), 2));
                    p.put("t",   (long) pt[3]);
                    arr.put(p);
                }
                o.put("trail_count", trail.size());
            }
            o.put("trail", arr);
            return o;
        } catch (Exception e) {
            Log.e(TAG, "mapJson", e);
            return new JSONObject();
        }
    }

    public JSONObject mapJsonPublic() { return mapJson(); }

    // ==============================================================
    // Flashlight helpers
    // ==============================================================

    public synchronized void setFlashlightMode(String mode) {
        config.flashlightMode = mode;
        if (flashlight != null) flashlight.setMode(mode);
        try { ConfigStore.save(getApplicationContext(), config); } catch (Exception ignored) {}
    }

    public synchronized void setFlashlightThreshold(int lux) {
        config.lightThreshold = lux;
        if (flashlight != null) flashlight.setThreshold(lux);
        try { ConfigStore.save(getApplicationContext(), config); } catch (Exception ignored) {}
    }

    public JSONObject flashlightJson() {
        try {
            JSONObject o = new JSONObject();
            if (flashlight == null) {
                o.put("available", false);
                o.put("running", false);
                o.put("lastError", "flashlight controller not initialized");
                return o;
            }
            o.put("available", flashlight.isAvailable());
            o.put("running", sensorRunning);
            o.put("mode", flashlight.getMode());
            o.put("on", flashlight.isOn());
            o.put("lux", flashlight.getLux());
            o.put("threshold", flashlight.getThreshold());
            o.put("lastError", flashlight.getLastError());
            o.put("appliedMode", flashlight.getAppliedMode());
            List<String> modes = flashlight.getSupportedModes();
            if (modes != null) o.put("supportedModes", new org.json.JSONArray(modes));
            return o;
        } catch (Exception e) { return new JSONObject(); }
    }

    // ==============================================================
    // statusJson
    // ==============================================================

    public JSONObject statusJson() {
        try {
            JSONObject o = new JSONObject();
            o.put("sensorRunning",   sensorRunning);
            o.put("lastCameraError", lastCameraError);
            o.put("previewWidth",    previewWidth);
            o.put("previewHeight",   previewHeight);

            o.put("httpPort",       config.httpPort);
            o.put("httpPortActive", sCurrentHttpPort);
            o.put("wsPort",         config.wsEnabled     ? config.wsPort     : -1);
            o.put("telnetPort",     config.telnetEnabled ? config.telnetPort : -1);

            o.put("httpActive",     httpServer   != null);
            o.put("wsActive",       wsServer     != null);
            o.put("telnetActive",   telnetServer != null);
            o.put("wsClients",      wsServer     == null ? 0 : wsServer.getClientCount());
            o.put("telnetClients",  telnetServer == null ? 0 : telnetServer.getClientCount());

            o.put("subscribers",    broadcaster.count());
            o.put("lastPublishMs",  broadcaster.getLastPublishMs());
            o.put("clientEnabled",  pushClient != null && pushClient.isRunning());

            Context ctx = getApplicationContext();
            o.put("localIp",  HttpUtil.getLocalIp(ctx));
            o.put("localIps", HttpUtil.getAllLocalIps(ctx));

            o.put("flashlight", flashlightJson());

            o.put("autoStart",        config.autoStart);
            o.put("sensorEnabledCfg", config.enabled);

            o.put("cameraPreviewEnabled",  config.cameraPreviewEnabled);
            o.put("cameraPreviewFps",      config.cameraPreviewFps);
            o.put("cameraPreviewQuality",  config.cameraPreviewQuality);
            o.put("cameraPreviewRotation", config.cameraPreviewRotation);

            o.put("cameraMountRotationDeg", config.cameraMountRotationDeg);
            o.put("invertDx",               config.invertDx);
            o.put("invertDy",               config.invertDy);

            o.put("publishOnlyOnChange",         config.publishOnlyOnChange);
            o.put("publishMotionThresholdPx",    config.publishMotionThresholdPx);
            o.put("publishRotationThresholdDeg", config.publishRotationThresholdDeg);
            o.put("publishHeartbeatMs",          config.publishHeartbeatMs);
            o.put("publishSeq",                  broadcaster.getPublishSeq());
            o.put("skippedReadings",             skippedReadings);

            OpticalFlowTracker.Result lr;
            synchronized (flowResultLock) { lr = lastFlowResult; }
            if (lr != null) {
                o.put("blocksPassedQuality", lr.blocksPassedQuality);
                o.put("blocksUsed",          lr.blocksUsed);
                o.put("blocksTotal",         lr.blocksTotal);
                o.put("confidenceLast",      lr.confidence);
                o.put("featureCount",        lr.featureCount);
            }
            o.put("flowMinVariance",       config.flowMinVariance);
            o.put("flowMinGradient",       config.flowMinGradient);
            o.put("flowSubpixel",          config.flowSubpixel);
            o.put("flowTemporalSmoothing", config.flowTemporalSmoothing);
            o.put("flowTemporalAlpha",     config.flowTemporalAlpha);
            o.put("flowMaxMotionPx",       config.flowMaxMotionPx);
            o.put("grayscaleOutput",       config.grayscaleOutput);
            o.put("position", positionJson());
            o.put("calib", calibStatusJson());
            o.put("rotation_calib", rotationCalibJson());
            o.put("trackerContrast",     config.trackerContrast);
            o.put("outputContrast",      config.outputContrast);
            o.put("cameraPreviewWidth",  config.cameraPreviewWidth);
            o.put("cameraPreviewHeight", config.cameraPreviewHeight);
            o.put("cameraSizes",         getCameraSizes());
            o.put("cameraId",            config.cameraId);
            o.put("cameraList",          getCameraList());
            boolean hasFrame;
            synchronized (snapshotLock) { hasFrame = (fullFrame != null); }
            o.put("hasFrame", hasFrame);
            return o;
        } catch (Exception e) {
            Log.e(TAG, "statusJson", e);
            return new JSONObject();
        }
    }

    private String buildStatusLine() {
        String ip = HttpUtil.getLocalIp(getApplicationContext());
        String base = "http://" + ip + ":" + sCurrentHttpPort + "/";
        String cam = "cam " + (config.cameraId >= 0 ? String.valueOf(config.cameraId) : "default");
        if (sensorRunning)
            return base + "  ·  " + previewWidth + "x" + previewHeight + "  ·  " + cam;
        if (!lastCameraError.isEmpty())
            return base + "  ·  камера: ошибка (" + cam + ")";
        return base + "  ·  камера остановлена (" + cam + ")";
    }

    // ==============================================================
    // Shutdown
    // ==============================================================

    private synchronized void shutdown() {
        if (uiHandler != null) {
            uiHandler.removeCallbacksAndMessages(null);
            uiHandler = null;
        }
        uiTick = null;
        stopSensorOnly();
        if (pushClient != null) { pushClient.stop(); pushClient = null; }
        stopServers();
    }

    // ==============================================================
    // Notification
    // ==============================================================

    private void createChannel() {
        if (Build.VERSION.SDK_INT >= 26) {
            NotificationChannel ch = new NotificationChannel(
                    CHANNEL_ID, "Optical Sensor", NotificationManager.IMPORTANCE_LOW);
            NotificationManager nm =
                    (NotificationManager) getSystemService(Context.NOTIFICATION_SERVICE);
            if (nm != null) nm.createNotificationChannel(ch);
        }
    }

    private Notification buildNotification(String text) {
        Intent i = new Intent(this, MainActivity.class);
        i.setFlags(Intent.FLAG_ACTIVITY_SINGLE_TOP);
        int piFlags = PendingIntent.FLAG_UPDATE_CURRENT;
        if (Build.VERSION.SDK_INT >= 23) piFlags |= PendingIntent.FLAG_IMMUTABLE;
        PendingIntent pi = PendingIntent.getActivity(this, 0, i, piFlags);
        Intent stop = new Intent(this, SensorService.class);
        stop.setAction(ACTION_STOP);
        PendingIntent stopPi = PendingIntent.getService(this, 1, stop, piFlags);

        String ip  = HttpUtil.getLocalIp(getApplicationContext());
        String url = "http://" + ip + ":" + sCurrentHttpPort + "/";

        NotificationCompat.Builder b = new NotificationCompat.Builder(this, CHANNEL_ID)
                .setContentTitle("OpticalRover  " + url)
                .setContentText(text)
                .setStyle(new NotificationCompat.BigTextStyle()
                        .bigText("Адрес: " + url + "\n" + text))
                .setSmallIcon(android.R.drawable.ic_menu_camera)
                .setContentIntent(pi)
                .setOngoing(true);
        b.addAction(android.R.drawable.ic_menu_close_clear_cancel, "Стоп", stopPi);
        return b.build();
    }

    private void updateNotification(String text) {
        NotificationManager nm =
                (NotificationManager) getSystemService(Context.NOTIFICATION_SERVICE);
        if (nm != null) nm.notify(NOTIF_ID, buildNotification(text));
    }

    public org.json.JSONArray getCameraList() {
        org.json.JSONArray arr = new org.json.JSONArray();
        try {
            int n = android.hardware.Camera.getNumberOfCameras();
            for (int i = 0; i < n; i++) {
                android.hardware.Camera.CameraInfo info =
                        new android.hardware.Camera.CameraInfo();
                android.hardware.Camera.getCameraInfo(i, info);
                String facing = "unknown";
                if (info.facing == android.hardware.Camera.CameraInfo.CAMERA_FACING_BACK)
                    facing = "back";
                else if (info.facing == android.hardware.Camera.CameraInfo.CAMERA_FACING_FRONT)
                    facing = "front";
                else if (Build.VERSION.SDK_INT >= 23 && info.facing == 2)
                    facing = "external";
                JSONObject o = new JSONObject();
                o.put("id", i);
                o.put("facing", facing);
                o.put("orientation", info.orientation);
                o.put("label", "Camera " + i + " (" + facing + ", " + info.orientation + "°)");
                arr.put(o);
            }
        } catch (Throwable t) {
            Log.w(TAG, "getCameraList: " + t.getMessage());
        }
        return arr;
    }
}