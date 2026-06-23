%% analyze_trial.m
%
% realtime_monitor.ino から取得したトライアルCSV
%   (elapsed_sec, angle_deg, flex_raw_adc, current_mA)
% を読み込み、基本的な可視化・統計・解析を行うMATLABスクリプトです。
%
% 使い方:
%   1. csv_path を解析したいCSVファイルのパスに変更する
%   2. 必要に応じて steady_t_start / steady_t_end を定常区間に合わせて変更する
%   3. MATLABでこのスクリプトを実行する (F5 or run)
%
% 出力:
%   - 3段グラフ（サーボ角度・曲げセンサ・電流 vs 時間）
%   - 電流の移動平均フィルタ後の比較
%   - 基本統計量（平均・最大・最小・標準偏差）
%   - 電流のパワースペクトル（周波数解析）
%   - サーボ駆動周期ごとの平均消費電力・エネルギーの推定
%   - 定常区間における、電流とサーボ周期運動の対応関係（位相平均・折り畳み解析）
%
% 各figureの末尾で tuneFigure を呼んでいます（お使いのカスタム見やすさ調整関数）。
% 関数の呼び出し形式（引数の有無など）がこちらの想定と違う場合は、
% 該当行（"tuneFigure;" としている箇所）を実際の使い方に合わせて書き換えてください。

clear; clc; close all;

%% ------------------------------------------------------------
%% 設定
%% ------------------------------------------------------------
csv_path = "trials/trial_20260623_162405.csv";  % ← 解析したいCSVのパスに変更してください

moving_avg_window = 5;       % 電流の移動平均フィルタの窓幅[サンプル数]
servo_frequency_hz = 1.25;   % ファームウェアのfrequencyHzと合わせる（周期ごとの解析に使用）

steady_t_start = 20.0;       % 定常区間の開始時刻[秒]
steady_t_end   = 30.0;       % 定常区間の終了時刻[秒]
n_phase_bins   = 24;         % 1周期を何ビンに分けて位相平均するか

%% ------------------------------------------------------------
%% データ読み込み
%% ------------------------------------------------------------
if ~isfile(csv_path)
    error("CSVファイルが見つかりません: %s", csv_path);
end

T = readtable(csv_path);

t = T.elapsed_sec;
angle = T.angle_deg;
flex = T.flex_raw_adc;
current = T.current_mA;

n_samples = numel(t);
duration_sec = t(end) - t(1);
fprintf("読み込んだサンプル数: %d\n", n_samples);
fprintf("トライアル時間: %.2f 秒\n", duration_sec);

if n_samples > 1
    dt_mean = mean(diff(t));
    fs_est = 1 / dt_mean;
    fprintf("推定サンプリング周波数: %.2f Hz (理論値50Hzと比較)\n", fs_est);
else
    fs_est = 50; % フォールバック
end

%% ------------------------------------------------------------
%% 基本統計量
%% ------------------------------------------------------------
fprintf("\n--- 基本統計量 ---\n");
print_stats("Servo angle [deg]", angle);
print_stats("Flex (raw ADC)", flex);
print_stats("Current [mA]", current);

%% ------------------------------------------------------------
%% 電流の移動平均フィルタ
%% ------------------------------------------------------------
current_filtered = movmean(current, moving_avg_window);

%% ------------------------------------------------------------
%% 図1: 3段グラフ（時系列）
%% ------------------------------------------------------------
figure('Name', 'Trial Overview', 'NumberTitle', 'off');

subplot(3,1,1);
plot(t, angle, 'b');
ylabel('Servo angle [deg]');
title('Servo angle / Flex sensor (raw) / Current');
grid on;
% 定常区間を背景でハイライト表示
xline(steady_t_start, '--k');
xline(steady_t_end, '--k');

subplot(3,1,2);
plot(t, flex, 'g');
ylabel('Flex (raw ADC)');
grid on;
xline(steady_t_start, '--k');
xline(steady_t_end, '--k');

subplot(3,1,3);
plot(t, current, 'r', 'Color', [1 0.6 0.6]); hold on;
plot(t, current_filtered, 'r', 'LineWidth', 1.5);
ylabel('Current [mA]');
xlabel('Time [s]');
legend('raw', sprintf('moving avg (window=%d)', moving_avg_window), 'Location', 'best');
grid on;
xline(steady_t_start, '--k', 'steady region');
xline(steady_t_end, '--k');

tuneFigure;

%% ------------------------------------------------------------
%% 図2: 電流のパワースペクトル（周波数解析）
%% ------------------------------------------------------------
figure('Name', 'Current Power Spectrum', 'NumberTitle', 'off');

current_detrended = current - mean(current);
N = length(current_detrended);
Y = fft(current_detrended);
P2 = abs(Y / N);
P1 = P2(1:floor(N/2)+1);
P1(2:end-1) = 2 * P1(2:end-1);
f = fs_est * (0:floor(N/2)) / N;

plot(f, P1);
xlabel('Frequency [Hz]');
ylabel('|Current amplitude| [mA]');
title('Current Power Spectrum (full trial)');
xlim([0, min(10, fs_est/2)]);
grid on;

hold on;
for k = 1:3
    xline(servo_frequency_hz * k, '--r', sprintf('%dx f_{servo}', k));
end
hold off;

tuneFigure;

%% ------------------------------------------------------------
%% サーボ駆動周期ごとの平均電流・推定エネルギー（トライアル全体）
%% ------------------------------------------------------------
fprintf("\n--- サーボ駆動周期ごとの解析（トライアル全体） ---\n");

period_sec = 1 / servo_frequency_hz;
n_periods = floor(duration_sec / period_sec);

if n_periods >= 1
    period_mean_current = nan(n_periods, 1);
    period_energy_mJ = nan(n_periods, 1);

    for k = 1:n_periods
        t_start = t(1) + (k-1) * period_sec;
        t_end = t(1) + k * period_sec;
        idx = (t >= t_start) & (t < t_end);

        if any(idx)
            period_mean_current(k) = mean(current(idx));
            period_energy_mJ(k) = trapz(t(idx), current(idx));
        end
    end

    figure('Name', 'Per-Cycle Current', 'NumberTitle', 'off');
    bar(1:n_periods, period_mean_current);
    xlabel('Servo cycle number');
    ylabel('Mean current [mA]');
    title(sprintf('Mean current per servo cycle (period = %.3f s)', period_sec));
    grid on;

    tuneFigure;

    fprintf("サーボ駆動周期数: %d\n", n_periods);
    fprintf("周期ごとの平均電流の平均: %.2f mA, 標準偏差: %.2f mA\n", ...
        mean(period_mean_current, 'omitnan'), std(period_mean_current, 'omitnan'));
else
    fprintf("トライアル時間が1周期に満たないため、周期ごとの解析はスキップしました。\n");
end

%% ------------------------------------------------------------
%% 定常区間(steady_t_start〜steady_t_end)の抽出
%% ------------------------------------------------------------
fprintf("\n--- 定常区間 (%.1f - %.1f 秒) の解析 ---\n", steady_t_start, steady_t_end);

idx_steady = (t >= steady_t_start) & (t <= steady_t_end);
n_steady = sum(idx_steady);

if n_steady < 2
    warning("定常区間にデータがほとんどありません。steady_t_start/endを確認してください。");
else
    t_steady = t(idx_steady);
    angle_steady = angle(idx_steady);
    flex_steady = flex(idx_steady);
    current_steady = current(idx_steady);

    fprintf("定常区間のサンプル数: %d\n", n_steady);
    print_stats("  [steady] Servo angle [deg]", angle_steady);
    print_stats("  [steady] Flex (raw ADC)", flex_steady);
    print_stats("  [steady] Current [mA]", current_steady);

    %% --------------------------------------------------------
    %% 電流とサーボ周期運動の対応: 周期で折り畳んで位相平均する
    %% --------------------------------------------------------
    % サーボの位相は trialStartMs (t=0) で phase=0 として駆動されているため、
    % 経過時間tから理論上の位相を直接計算できる:
    %   phase(t) = mod(2*pi*frequencyHz*t, 2*pi)
    % これを使って、定常区間の各サンプルを「1周期内のどの位相か」にマッピングし、
    % 同じ位相に属するサンプルをまとめて平均する（位相平均、cycle-averaging）。

    phase_rad = mod(2*pi*servo_frequency_hz*t_steady, 2*pi);
    phase_deg = rad2deg(phase_rad);

    % 位相をビンに分けて、各ビンごとに電流・角度・曲げセンサの平均と標準偏差を求める
    bin_edges = linspace(0, 360, n_phase_bins+1);
    bin_centers = (bin_edges(1:end-1) + bin_edges(2:end)) / 2;

    current_bin_mean = nan(n_phase_bins, 1);
    current_bin_std  = nan(n_phase_bins, 1);
    angle_bin_mean   = nan(n_phase_bins, 1);
    flex_bin_mean    = nan(n_phase_bins, 1);

    for b = 1:n_phase_bins
        in_bin = phase_deg >= bin_edges(b) & phase_deg < bin_edges(b+1);
        if any(in_bin)
            current_bin_mean(b) = mean(current_steady(in_bin));
            current_bin_std(b)  = std(current_steady(in_bin));
            angle_bin_mean(b)   = mean(angle_steady(in_bin));
            flex_bin_mean(b)    = mean(flex_steady(in_bin));
        end
    end

    % --- 図3: 位相に対する電流・角度の重ね描き（左右2軸） ---
    figure('Name', 'Current vs Servo Phase (steady region)', 'NumberTitle', 'off');

    yyaxis left;
    % 生データの散布図（折り畳み済み、全周期分を重ねて表示）
    scatter(phase_deg, current_steady, 8, [1 0.7 0.7], 'filled'); hold on;
    errorbar(bin_centers, current_bin_mean, current_bin_std, '-o', ...
        'Color', 'r', 'LineWidth', 1.5, 'MarkerFaceColor', 'r');
    ylabel('Current [mA]');

    yyaxis right;
    plot(bin_centers, angle_bin_mean, '-', 'Color', 'b', 'LineWidth', 1.5);
    ylabel('Servo angle [deg]');

    xlabel('Phase within servo cycle [deg]  (0=trial start phase)');
    title(sprintf('Current vs servo phase, folded over %.1f-%.1fs (%.2f Hz)', ...
        steady_t_start, steady_t_end, servo_frequency_hz));
    xlim([0, 360]);
    legend({'current (raw, all cycles overlaid)', 'current (phase-binned mean ± std)', ...
        'servo angle (phase-binned mean)'}, 'Location', 'best');
    grid on;

    tuneFigure;

    % --- 相関・位相ズレの確認 ---
    % 角度とフレックス、電流の間にどれくらいの時間ズレ（位相ズレ）があるかを
    % 相互相関(xcorr)で確認する。ピークの位置がそのままズレ時間に対応する。
    current_centered = current_steady - mean(current_steady);
    angle_centered = angle_steady - mean(angle_steady);

    [xc, lags] = xcorr(current_centered, angle_centered, round(fs_est * period_sec), 'coeff');
    lag_time_sec = lags / fs_est;

    figure('Name', 'Cross-correlation: Current vs Angle', 'NumberTitle', 'off');
    plot(lag_time_sec, xc, 'LineWidth', 1.2);
    xlabel('Lag [s] (positive: current lags angle)');
    ylabel('Normalized cross-correlation');
    title('Cross-correlation between current and servo angle (steady region)');
    grid on;

    [peak_val, peak_idx] = max(xc);
    peak_lag_sec = lag_time_sec(peak_idx);
    xline(peak_lag_sec, '--r', sprintf('peak lag = %.3f s', peak_lag_sec));

    tuneFigure;

    fprintf("電流と角度の相互相関ピーク: lag = %.4f s (correlation = %.3f)\n", ...
        peak_lag_sec, peak_val);
    fprintf("  (lag>0: 電流の変化が角度より遅れている / lag<0: 電流が先行している)\n");
end

%% ------------------------------------------------------------
%% ヘルパー関数
%% ------------------------------------------------------------
function print_stats(label, data)
    fprintf("%-28s mean=%.3f, min=%.3f, max=%.3f, std=%.3f\n", ...
        label, mean(data), min(data), max(data), std(data));
end