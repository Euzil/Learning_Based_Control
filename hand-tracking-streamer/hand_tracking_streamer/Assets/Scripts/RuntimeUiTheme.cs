using TMPro;
using UnityEngine;
using UnityEngine.UI;

public static class RuntimeUiTheme
{
    public static void Apply(GameObject menuPanel, Button startButton, TextMeshProUGUI statusText, TextMeshProUGUI versionText, RuntimeUiThemeSettings settings)
    {
        if (menuPanel == null)
        {
            return;
        }

        if (settings == null)
        {
            settings = RuntimeUiThemeSettings.Default();
        }

        StylePanel(menuPanel, settings);
        StyleLayout(menuPanel, settings);
        StyleGraphics(menuPanel, settings);
        StyleText(menuPanel, statusText, versionText, settings);
        StyleControls(menuPanel, settings);
        StyleStartButton(startButton, settings);
    }

    public static Color StatusColor(Color requested, RuntimeUiThemeSettings settings)
    {
        if (settings == null)
        {
            settings = RuntimeUiThemeSettings.Default();
        }

        if (Approximately(requested, Color.red)) return settings.errorColor;
        if (Approximately(requested, Color.yellow)) return settings.warningColor;
        if (Approximately(requested, Color.green)) return settings.successColor;
        return requested;
    }

    private static void StylePanel(GameObject menuPanel, RuntimeUiThemeSettings settings)
    {
        var image = menuPanel.GetComponent<Image>();
        if (image != null)
        {
            image.color = settings.panelColor;
            image.type = Image.Type.Sliced;
        }

        var rect = menuPanel.GetComponent<RectTransform>();
        if (rect != null)
        {
            rect.sizeDelta = new Vector2(0f, 0f);
        }
    }

    private static void StyleLayout(GameObject menuPanel, RuntimeUiThemeSettings settings)
    {
        var layout = menuPanel.GetComponent<VerticalLayoutGroup>();
        if (layout == null) return;

        layout.padding = new RectOffset(settings.horizontalPadding, settings.horizontalPadding, settings.verticalPadding, settings.verticalPadding);
        layout.spacing = settings.controlSpacing;
        layout.childAlignment = TextAnchor.UpperCenter;
        layout.childForceExpandWidth = true;
        layout.childControlWidth = true;
        layout.childControlHeight = false;
    }

    private static void StyleGraphics(GameObject menuPanel, RuntimeUiThemeSettings settings)
    {
        foreach (var image in menuPanel.GetComponentsInChildren<Image>(true))
        {
            string objectName = image.gameObject.name;

            if (objectName == "Panel")
            {
                continue;
            }

            if (objectName == "Background" || objectName == "Checkmark")
            {
                image.color = objectName == "Checkmark" ? settings.primaryColor : settings.fieldColor;
                continue;
            }

            if (objectName.Contains("Dropdown") || objectName.Contains("InputField"))
            {
                image.color = settings.fieldColor;
                continue;
            }

            image.color = settings.surfaceColor;
        }
    }

    private static void StyleText(GameObject menuPanel, TextMeshProUGUI statusText, TextMeshProUGUI versionText, RuntimeUiThemeSettings settings)
    {
        foreach (var text in menuPanel.GetComponentsInChildren<TMP_Text>(true))
        {
            string objectName = text.gameObject.name;

            text.color = objectName.Contains("Placeholder") ? settings.mutedTextColor : settings.textColor;
            text.fontStyle &= ~FontStyles.Bold;
            text.enableWordWrapping = true;

            if (text.text.Contains("Hand Tracking Streamer"))
            {
                text.fontSize = settings.titleFontSize;
                text.fontStyle |= FontStyles.Bold;
                text.color = settings.textColor;
            }
            else if (objectName.Contains("Label") || objectName.Contains("Text"))
            {
                text.fontSize = Mathf.Clamp(text.fontSize, settings.bodyFontSize, settings.bodyFontSize + 6f);
            }
        }

        if (statusText != null)
        {
            statusText.fontStyle = FontStyles.Bold;
            statusText.color = settings.successColor;
        }

        if (versionText != null)
        {
            versionText.color = settings.mutedTextColor;
            versionText.fontSize = Mathf.Min(versionText.fontSize, settings.bodyFontSize);
        }
    }

    private static void StyleControls(GameObject menuPanel, RuntimeUiThemeSettings settings)
    {
        foreach (var selectable in menuPanel.GetComponentsInChildren<Selectable>(true))
        {
            var colors = selectable.colors;
            colors.normalColor = settings.fieldColor;
            colors.highlightedColor = settings.surfaceSoftColor;
            colors.selectedColor = settings.surfaceSoftColor;
            colors.pressedColor = settings.primaryColor;
            colors.disabledColor = settings.disabledColor;
            colors.colorMultiplier = 1f;
            colors.fadeDuration = 0.08f;
            selectable.colors = colors;
        }

        foreach (var layoutElement in menuPanel.GetComponentsInChildren<LayoutElement>(true))
        {
            if (layoutElement.gameObject.name.Contains("Btn_Start"))
            {
                layoutElement.preferredHeight = settings.startButtonHeight;
            }
            else if (!layoutElement.ignoreLayout && layoutElement.preferredHeight > 0f)
            {
                layoutElement.preferredHeight = Mathf.Max(layoutElement.preferredHeight, settings.controlMinHeight);
            }
        }
    }

    private static void StyleStartButton(Button startButton, RuntimeUiThemeSettings settings)
    {
        if (startButton == null) return;

        var image = startButton.GetComponent<Image>();
        if (image != null)
        {
            image.color = settings.primaryColor;
        }

        var colors = startButton.colors;
        colors.normalColor = settings.primaryColor;
        colors.highlightedColor = settings.primaryGlowColor;
        colors.selectedColor = settings.primaryGlowColor;
        colors.pressedColor = settings.primaryPressedColor;
        colors.disabledColor = settings.disabledColor;
        colors.colorMultiplier = 1f;
        colors.fadeDuration = 0.08f;
        startButton.colors = colors;

        var label = startButton.GetComponentInChildren<TMP_Text>(true);
        if (label != null)
        {
            label.color = settings.buttonTextColor;
            label.fontStyle = FontStyles.Bold;
            label.fontSize = settings.buttonFontSize;
        }

        var breather = startButton.GetComponent<UIButtonBreather>();
        if (breather != null)
        {
            breather.Configure(settings.primaryColor, settings.primaryGlowColor, settings.buttonPulseSpeed);
        }
    }

    private static bool Approximately(Color left, Color right)
    {
        const float tolerance = 0.02f;
        return Mathf.Abs(left.r - right.r) < tolerance
            && Mathf.Abs(left.g - right.g) < tolerance
            && Mathf.Abs(left.b - right.b) < tolerance;
    }

    private static Color Hex(string value, float alpha)
    {
        ColorUtility.TryParseHtmlString("#" + value, out Color color);
        color.a = alpha;
        return color;
    }

    public static Color ColorFromHex(string value, float alpha)
    {
        return Hex(value, alpha);
    }
}

[System.Serializable]
public class RuntimeUiThemeSettings
{
    [Header("Main Colors")]
    public Color panelColor;
    public Color surfaceColor;
    public Color surfaceSoftColor;
    public Color fieldColor;
    public Color primaryColor;
    public Color primaryGlowColor;
    public Color primaryPressedColor;
    public Color disabledColor;

    [Header("Text Colors")]
    public Color textColor;
    public Color mutedTextColor;
    public Color buttonTextColor;

    [Header("Status Colors")]
    public Color successColor;
    public Color warningColor;
    public Color errorColor;

    [Header("Sizing")]
    [Range(12, 64)] public int horizontalPadding = 32;
    [Range(12, 64)] public int verticalPadding = 28;
    [Range(6f, 32f)] public float controlSpacing = 14f;
    [Range(40f, 96f)] public float controlMinHeight = 56f;
    [Range(48f, 120f)] public float startButtonHeight = 78f;
    [Range(18f, 48f)] public float titleFontSize = 34f;
    [Range(14f, 32f)] public float bodyFontSize = 20f;
    [Range(18f, 42f)] public float buttonFontSize = 30f;
    [Range(0f, 4f)] public float buttonPulseSpeed = 1.2f;

    public static RuntimeUiThemeSettings Default()
    {
        return new RuntimeUiThemeSettings
        {
            panelColor = RuntimeUiTheme.ColorFromHex("172126", 0.88f),
            surfaceColor = RuntimeUiTheme.ColorFromHex("223139", 0.92f),
            surfaceSoftColor = RuntimeUiTheme.ColorFromHex("2d424b", 0.82f),
            fieldColor = RuntimeUiTheme.ColorFromHex("10191d", 0.86f),
            primaryColor = RuntimeUiTheme.ColorFromHex("21d0a7", 1f),
            primaryGlowColor = RuntimeUiTheme.ColorFromHex("7fffd4", 1f),
            primaryPressedColor = RuntimeUiTheme.ColorFromHex("14a889", 1f),
            disabledColor = RuntimeUiTheme.ColorFromHex("536369", 0.55f),
            textColor = RuntimeUiTheme.ColorFromHex("eef8f5", 1f),
            mutedTextColor = RuntimeUiTheme.ColorFromHex("9cb3b6", 1f),
            buttonTextColor = RuntimeUiTheme.ColorFromHex("07130f", 1f),
            successColor = RuntimeUiTheme.ColorFromHex("46e6aa", 1f),
            warningColor = RuntimeUiTheme.ColorFromHex("ffd166", 1f),
            errorColor = RuntimeUiTheme.ColorFromHex("ff6b6b", 1f),
        };
    }
}
