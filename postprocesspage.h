#ifndef POSTPROCESSPAGE_H
#define POSTPROCESSPAGE_H

#include <QWidget>
#include <QVBoxLayout>
#include <QComboBox>
#include <QLabel>
#include <QPushButton>
#include <QSlider> // Added for QSlider
#include <QLineEdit>
#include <QFileDialog>
#include <QMessageBox>
#include <QChartView>
#include <QChart>
#include <QLineSeries>
#include <QValueAxis>
#include <vector>
#include <map>
#include <string>
#include <cmath>
#include <algorithm>
#include <numeric>
#include <QRadioButton>
#include <QButtonGroup>

class PostProcessPage : public QWidget {
    Q_OBJECT

public:
    explicit PostProcessPage(QWidget *parent = nullptr, bool isDarkTheme = true);
    void applyTheme();

private slots:
    void populateFiles();
    void loadFile(int index);
    void updatePlot();
    void trimData();
    void toggleFilterFrame();
    void applyFilter();
    void saveToHtml();
    void startTrimming(QPointF point);
    void updateTrimming(QPointF point);
    void endTrimming(QPointF point);
    void undoTrim();
    void updateCutoff(int value); // New slot for cutoff slider
    void updateOrder(int value); // New slot for order slider
    void undoFilter(); // New slot for undoing filter
    void onModeChanged();

private:
    std::vector<double> applyButterworthFilter(const std::vector<double>& signal, double cutoff, int order, double fs, bool highpass);
    std::vector<double> fir_low_pass_filter(const std::vector<double>& signal, double cutoff_frequency, double sample_rate, int filter_length);
    std::vector<double> fir_high_pass_filter(const std::vector<double>& signal, double cutoff_frequency, double sample_rate, int filter_length);
    void calculateFs();

    QComboBox *fileCombo;
    QLabel *fileLabel;
    QComboBox *imuCombo;
    QComboBox *paramCombo;
    QPushButton *trimBtn;
    QPushButton *filterBtn;
    QWidget *filterFrame;
    QComboBox *typeCombo;
    QSlider *cutoffSlider; // Replaced QDoubleSpinBox with QSlider
    QSlider *orderSlider; // Replaced QSpinBox with QSlider
    QLabel *cutoffValueLabel; // New label to display cutoff value
    QLabel *orderValueLabel; // New label to display order value
    QLineEdit *titleEdit;
    QPushButton *applyBtn;
    QPushButton *saveBtn;
    QChartView *chartView;
    QChart *chart;
    QLineSeries *series;
    QLineSeries *filteredSeries;

    bool isDarkTheme;
    QString currentFile;
    int currentIMU;
    QString currentParam;
    std::vector<double> originalTimes; // in seconds
    std::vector<double> originalValues;
    std::vector<double> currentTimes;
    std::vector<double> currentValues;
    double fs; // sampling frequency
    std::map<QString, int> paramMap;

    enum XYMetric { Roll=0, Pitch=1, Yaw=2 };
    XYMetric currentXYMetric() const { return static_cast<XYMetric>(angleCombo->currentIndex()); }
    bool xyMode() const { return xyRadio && xyRadio->isChecked(); }

    bool isTrimming;
    QRectF trimRect;
    QPointF trimStartPoint;
    std::vector<double> trimmedTimesBackup;
    std::vector<double> trimmedValuesBackup;

    void handlePointHover(const QPointF &point, bool state);

    QPushButton *undoFilterBtn; // New button for undoing filter

    // متغیرهای جدید
    QPushButton *movingAverageBtn;
    QWidget *movingAverageFrame;
    QSlider *windowSizeSlider;
    QPushButton *applyMovingBtn; // دکمه Apply جدا برای moving average
    QLabel *windowSizeValueLabel;
    QLineSeries *movingAverageSeries;
    std::vector<double> movingAverageValues; // برای ذخیره داده‌های میانگین متحرک

    std::vector<double> filteredValues; // برای ذخیره داده‌های فیلترشده (فقط Time mode)

    // UI for modes
    QButtonGroup *modeGroup;
    QRadioButton *timeRadio;
    QRadioButton *xyRadio;
    QWidget *timeSelectWidget;
    QWidget *xySelectWidget;

    // UI for XY mode
    QComboBox *jointXCombo;
    QComboBox *jointYCombo;
    QComboBox *angleCombo;
    QPushButton *applyXYBtn;
    QStringList jointNames;

    // Series for XY
    QLineSeries *xySeries;
    QLineSeries *xyFilteredSeries;
    QLineSeries *xyMovingSeries;

    // Data for XY
    std::vector<double> angleX;
    std::vector<double> angleY;
    std::vector<double> filteredY;
    std::vector<double> movingY;
    std::vector<double> angleXBackup;
    std::vector<double> angleYBackup;

    // اسلات‌های جدید
    void toggleMovingAverageFrame();
    void applyMovingAverage();
    void updateWindowSize(int value);
    void undoMovingAverage();

    // متد جدید برای محاسبه میانگین متحرک
    std::vector<double> calculateMovingAverage(const std::vector<double>& signal, int windowSize);

    void loadXYData(int jointXIdx, int jointYIdx, XYMetric metric);
    QString getAngleName(XYMetric metric) const;
    void updateXYPlot();
};

#endif // POSTPROCESSPAGE_H
