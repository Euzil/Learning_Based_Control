using System.Collections.Generic;
using TMPro;
using UnityEngine;
using UnityEngine.UI;

public class VideoPanelRenderer : MonoBehaviour
{
    [SerializeField] private GameObject panelRoot;
    [SerializeField] private RawImage targetRawImage;
    [SerializeField] private Renderer targetRenderer;
    [SerializeField] private TextMeshProUGUI placeholderText;
    [SerializeField, Range(30, 100)] private float verticalViewDegrees = 70;
    [SerializeField, Min(0.5f)] private float viewDistance = 1.8f;
    private bool robotVisionActive;
    private readonly Dictionary<OVRPassthroughLayer, bool> hiddenStates = new Dictionary<OVRPassthroughLayer, bool>();
    private readonly Dictionary<Camera, (CameraClearFlags flags, Color color)> cameraStates = new Dictionary<Camera, (CameraClearFlags, Color)>();
    private Texture currentTexture;

    public void SetTexture(Texture texture)
    {
        currentTexture = texture;
        if (targetRawImage != null) targetRawImage.texture = texture;
        if (targetRenderer != null) targetRenderer.material.mainTexture = texture;
        if (placeholderText != null) placeholderText.text = texture == null ? "No Video" : string.Empty;
        if (robotVisionActive) FitView();
    }

    private void FitView()
    {
        if (panelRoot == null || targetRawImage == null || currentTexture == null) return;
        var rig = FindFirstObjectByType<OVRCameraRig>();
        var eye = rig != null ? rig.centerEyeAnchor : Camera.main != null ? Camera.main.transform : null;
        if (eye == null) return;
        var canvas = panelRoot.GetComponent<Canvas>();
        var rect = panelRoot.GetComponent<RectTransform>();
        if (canvas == null || rect == null) return;
        canvas.renderMode = RenderMode.WorldSpace;
        canvas.worldCamera = eye.GetComponent<Camera>();
        rect.SetParent(eye, false);
        rect.localPosition = new Vector3(0, 0, viewDistance);
        rect.localRotation = Quaternion.identity;
        float height = Mathf.Max(1, currentTexture.height);
        rect.sizeDelta = new Vector2(currentTexture.width, height);
        float scale = 2 * viewDistance * Mathf.Tan(verticalViewDegrees * Mathf.Deg2Rad / 2) / height;
        rect.localScale = Vector3.one * scale;
        var image = targetRawImage.rectTransform;
        image.anchorMin = Vector2.zero;
        image.anchorMax = Vector2.one;
        image.offsetMin = image.offsetMax = Vector2.zero;
        targetRawImage.color = Color.white;
        targetRawImage.raycastTarget = false;
    }

    public void SetRobotVisionActive(bool active)
    {
        active = active && currentTexture != null;
        if (active == robotVisionActive)
        {
            SetVisible(active);
            return;
        }
        robotVisionActive = active;
        if (active)
        {
            hiddenStates.Clear();
            foreach (var layer in FindObjectsByType<OVRPassthroughLayer>(FindObjectsInactive.Include, FindObjectsSortMode.None))
            {
                hiddenStates[layer] = layer.hidden;
                layer.hidden = true; // Preserve tracking and the passthrough component's enabled state.
            }
            cameraStates.Clear();
            var rig = FindFirstObjectByType<OVRCameraRig>();
            var cameras = rig != null ? rig.GetComponentsInChildren<Camera>(true) : Camera.allCameras;
            foreach (var camera in cameras)
            {
                cameraStates[camera] = (camera.clearFlags, camera.backgroundColor);
                camera.clearFlags = CameraClearFlags.SolidColor;
                camera.backgroundColor = Color.black;
            }
            FitView();
        }
        else RestoreOriginalView();
        SetVisible(active);
    }

    private void RestoreOriginalView()
    {
        foreach (var entry in hiddenStates) if (entry.Key != null) entry.Key.hidden = entry.Value;
        foreach (var entry in cameraStates)
            if (entry.Key != null)
            {
                entry.Key.clearFlags = entry.Value.flags;
                entry.Key.backgroundColor = entry.Value.color;
            }
        hiddenStates.Clear();
        cameraStates.Clear();
        robotVisionActive = false;
    }

    public void SetStatus(string text) { if (placeholderText != null) placeholderText.text = text; }
    public void Clear() { SetRobotVisionActive(false); SetTexture(null); }
    public void SetVisible(bool visible) { if (panelRoot != null) panelRoot.SetActive(visible); }
    private void OnDisable() { RestoreOriginalView(); SetVisible(false); }
    private void OnDestroy() { RestoreOriginalView(); }
}
