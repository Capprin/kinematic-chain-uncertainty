function color = color_gradient(high_color, low_color, high_value, low_value, current_value)
    prop = (current_value - low_value) / (high_value - low_value);
    color = low_color + prop*(high_color - low_color);
end