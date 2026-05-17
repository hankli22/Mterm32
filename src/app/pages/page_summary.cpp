#include "app/pages.h"
#include "app/menu.h"
#include "svc/gps.h"
#include "svc/config.h"

void drawTrackMap(Canvas& cv, int ox, int oy, int lapIdx) {
  int start = 0;
  int end = GPSCalc::trackPointsCount - 1;

  if (lapIdx > 0 && lapIdx <= GPSCalc::laps) {
    start = GPSCalc::lapHistory[lapIdx - 1].trackStartIdx;
    end = GPSCalc::lapHistory[lapIdx - 1].trackEndIdx;
  }

  if (end - start < 1 || start < 0) return;

  float minX = 9999, maxX = -9999, minY = 9999, maxY = -9999;
  for (int i = 0; i < GPSCalc::trackPointsCount; i++) {
    if (GPSCalc::trackX[i] < minX) minX = GPSCalc::trackX[i];
    if (GPSCalc::trackX[i] > maxX) maxX = GPSCalc::trackX[i];
    if (GPSCalc::trackY[i] < minY) minY = GPSCalc::trackY[i];
    if (GPSCalc::trackY[i] > maxY) maxY = GPSCalc::trackY[i];
  }
  float maxDim = max(maxX - minX, maxY - minY);
  float scale = 44.0f / (maxDim > 0.1 ? maxDim : 1.0f);

  for (int i = start + 1; i <= end; i++) {
    int px1 = ox + (GPSCalc::trackX[i - 1] - minX) * scale;
    int py1 = oy + 44 - (GPSCalc::trackY[i - 1] - minY) * scale;
    int px2 = ox + (GPSCalc::trackX[i] - minX) * scale;
    int py2 = oy + 44 - (GPSCalc::trackY[i] - minY) * scale;
    cv.drawLine(px1, py1, px2, py2);
  }
  cv.drawFrame(ox - 2, oy - 2, 48, 48);
}

void drawSummary(Canvas& cv) {
  cv.setFont(u8g2_font_6x10_tf);
  cv.drawStr(2, 10, "Summary");

  float avgPace = 0;
  if (GPSCalc::totalDistance > 10 && GPSCalc::durationSec > 0) {
    float speedKmh = (GPSCalc::totalDistance / GPSCalc::durationSec) * 3.6f;
    if (speedKmh > 1.0f) avgPace = 60.0f / speedKmh;
  }

  cv.setCursor(60, 10);
  if (avgPace > 0) {
    cv.print("Avg:");
    cv.print((int)avgPace);
    cv.print("'");
    int avgSec = (int)((avgPace - (int)avgPace) * 60);
    if (avgSec < 10) cv.print("0");
    cv.print(avgSec);
    cv.print("\"");
  }

  int slowestLapIdx = -1;
  int maxTime = 0;
  for (int i = 0; i < GPSCalc::laps; i++) {
    if (GPSCalc::lapHistory[i].timeSec > maxTime) {
      maxTime = GPSCalc::lapHistory[i].timeSec;
      slowestLapIdx = i;
    }
  }

  for (int i = 0; i < GPSCalc::laps && i < 4; i++) {
    int y = 24 + i * 10;
    if (MenuManager::viewLapIdx == i + 1) cv.drawStr(0, y, ">");

    if (i == slowestLapIdx) {
      cv.drawBox(8, y - 8, 55, 10);
      cv.setDrawColor(0);
    }

    cv.setCursor(10, y);
    cv.print("L");
    cv.print(i + 1);
    cv.print(": ");
    cv.print((int)GPSCalc::lapHistory[i].pace);
    cv.print("'");
    int lapSec = (int)((GPSCalc::lapHistory[i].pace - (int)GPSCalc::lapHistory[i].pace) * 60);
    if (lapSec < 10) cv.print("0");
    cv.print(lapSec);
    cv.print("\"");

    cv.setDrawColor(1);
  }

  if (sysCfg.draw_track) drawTrackMap(cv, 70, 15, MenuManager::viewLapIdx);

  cv.setFont(u8g2_font_4x6_tf);
  cv.drawStr(70, 62, MenuManager::viewLapIdx == 0 ? "VIEW: ALL" : "VIEW: LAP");
}

void drawFullMapHUD(Canvas& cv) {
  const int W = 128, H = 64;

  for (int gx = 6; gx < W; gx += 12)
    for (int gy = 6; gy < H; gy += 12)
      cv.setPixel(gx, gy);

  int start = 0;
  int end = GPSCalc::trackPointsCount - 1;

  const int s = 5;
  cv.drawLine(0, 0, s, 0); cv.drawLine(0, 0, 0, s);
  cv.drawLine(W - 1, 0, W - 1 - s, 0); cv.drawLine(W - 1, 0, W - 1, s);
  cv.drawLine(0, H - 1, s, H - 1); cv.drawLine(0, H - 1, 0, H - 1 - s);
  cv.drawLine(W - 1, H - 1, W - 1 - s, H - 1); cv.drawLine(W - 1, H - 1, W - 1, H - 1 - s);

  if (end - start < 1) {
    cv.setFont(u8g2_font_5x8_tf);
    cv.drawStr(W / 2 - 22, H / 2, "NO TRACK");
  } else {
    float minX = GPSCalc::trackX[start], maxX = GPSCalc::trackX[start];
    float minY = GPSCalc::trackY[start], maxY = GPSCalc::trackY[start];
    for (int i = start + 1; i <= end; i++) {
      float tx = GPSCalc::trackX[i], ty = GPSCalc::trackY[i];
      if (tx < minX) minX = tx;
      if (tx > maxX) maxX = tx;
      if (ty < minY) minY = ty;
      if (ty > maxY) maxY = ty;
    }

    float rangeX = maxX - minX, rangeY = maxY - minY;
    const int M = 4;
    float sx = (W - M * 2) / (rangeX > 0.1f ? rangeX : 1.0f);
    float sy = (H - M * 2) / (rangeY > 0.1f ? rangeY : 1.0f);
    float scale = sx < sy ? sx : sy;

    int offX = M + (W - M * 2 - rangeX * scale) / 2;
    int offY = M + (H - M * 2 - rangeY * scale) / 2;

    for (int i = start + 1; i <= end; i++) {
      int x1 = offX + (GPSCalc::trackX[i - 1] - minX) * scale;
      int y1 = offY + (maxY - GPSCalc::trackY[i - 1]) * scale;
      int x2 = offX + (GPSCalc::trackX[i] - minX) * scale;
      int y2 = offY + (maxY - GPSCalc::trackY[i]) * scale;
      cv.drawLine(x1, y1, x2, y2);
    }

    // start marker
    int sx0 = offX + (GPSCalc::trackX[start] - minX) * scale;
    int sy0 = offY + (maxY - GPSCalc::trackY[start]) * scale;
    cv.drawCircle(sx0, sy0, 2);

    // blinking crosshair at current position
    int curX = offX + (GPSCalc::trackX[end] - minX) * scale;
    int curY = offY + (maxY - GPSCalc::trackY[end]) * scale;
    if ((millis() / 300) & 1) {
      cv.drawLine(curX - 4, curY, curX + 4, curY);
      cv.drawLine(curX, curY - 4, curX, curY + 4);
    }

    // smart scale bar
    static const int NICE[] = {10, 20, 50, 100, 200, 500, 1000, 2000, 5000};
    float dist20 = 20.0f / scale;
    int chosen = NICE[0];
    float best = abs(NICE[0] - dist20);
    for (int i = 1; i < 9; i++) {
      float d = abs(NICE[i] - dist20);
      if (d < best) { best = d; chosen = NICE[i]; }
    }

    int barW = chosen * scale;
    if (barW < 8) barW = 8;
    if (barW > 60) barW = 60;

    int barX = W - barW - 10, barY = H - 9;
    cv.drawLine(barX, barY, barX + barW, barY);
    cv.drawLine(barX, barY - 3, barX, barY + 1);
    cv.drawLine(barX + barW, barY - 3, barX + barW, barY + 1);

    cv.setFont(u8g2_font_4x6_tf);
    char buf[10];
    if (chosen >= 1000) snprintf(buf, sizeof(buf), "%d.%dkm", chosen / 1000, (chosen % 1000) / 100);
    else snprintf(buf, sizeof(buf), "%dm", chosen);
    cv.drawStr(barX + 2, barY - 4, buf);
  }

  // HUD overlays
  cv.setDrawColor(1);
  cv.drawBox(0, 0, 35, 13);
  cv.setDrawColor(0);
  cv.setFont(u8g2_font_6x12_tn);
  cv.setCursor(2, 11);
  cv.print(GPSCalc::currentSpeed, 1);
  cv.setFont(u8g2_font_4x6_tf);
  cv.drawStr(24, 10, "K/H");

  cv.setDrawColor(1);
  cv.drawBox(0, H - 12, 44, 12);
  cv.setDrawColor(0);
  cv.setFont(u8g2_font_5x7_tf);
  cv.setCursor(2, H - 3);
  cv.print((int)GPSCalc::totalDistance);
  cv.print(" m");

  cv.setDrawColor(1);
  cv.drawBox(W - 40, 0, 40, 11);
  cv.setDrawColor(0);
  cv.setFont(u8g2_font_5x7_tf);
  int min = GPSCalc::durationSec / 60;
  int sec = GPSCalc::durationSec % 60;
  char tBuf[8];
  snprintf(tBuf, sizeof(tBuf), "%02d:%02d", min, sec);
  cv.drawStr(W - 36, 9, tBuf);

  cv.setDrawColor(1);
}
