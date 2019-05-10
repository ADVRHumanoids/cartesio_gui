import QtQuick 2.4

SignedBarForm {

    property double from: -1.0
    property double to: 1.0
    property double value: 0.5

    onValueChanged:
    {

        height_ratio = Math.abs(value / (to - from))

        var h_zero = to / (to - from)

        if(value < 0)
        {
            y_ratio = h_zero
        }
        else
        {
            y_ratio = (h_zero - value/(to - from))
        }
    }






}
