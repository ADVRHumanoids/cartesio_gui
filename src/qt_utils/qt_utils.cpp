#include <cartesio_gui/qt_utils/qt_utils.h>
using namespace cartesio_gui::qt_utils;

BoxedSliderWidget::BoxedSliderWidget(QWidget* parent_widget):
_parent_widget(parent_widget)
{
    /* Create GUI layout */
    auto * ui = LoadUiFile(this);

    auto * layout = new QVBoxLayout;
    layout->addWidget(ui);

    setLayout(layout);

   _slider = findChild<QSlider *>("horizontalSlider");
   _spin_box = findChild<QDoubleSpinBox *>("doubleSpinBox");
   _label = findChild<QLabel *>("label");

   this->setLimits(-100, 100);
   this->setValue(0.0);

   connect(_slider, &QSlider::valueChanged,
               this, &BoxedSliderWidget::on_slider_value_changed);

   connect(_spin_box, &QDoubleSpinBox::editingFinished,
               this, &BoxedSliderWidget::on_box_value_changed);
}

void BoxedSliderWidget::on_slider_value_changed(int value)
{
    _value = value;
    _spin_box->setValue(_value);

    on_value_changed_call_back(_value);
}

void BoxedSliderWidget::on_box_value_changed()
{
    _value = _spin_box->value();
    _slider->setValue(_value);
}

void BoxedSliderWidget::setLimits(int min, int max)
{
    _min = min;
    _max = max;

    _slider->setMinimum(_min);
    _slider->setMaximum(_max);

    _spin_box->setMinimum(_min);
    _spin_box->setMaximum(_max);
}

void BoxedSliderWidget::setValue(int value)
{
    _value = value;
    _spin_box->setValue(_value);
    _slider->setValue(_value);
}

void BoxedSliderWidget::setLable(const QString& label)
{
    _label->setText(label);
}
