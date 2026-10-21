#if UNITY_EDITOR
using System;
using System.Reflection;
using UnityEditor;
using UnityEditor.SceneManagement;
using UnityEngine;
using UnityEngine.UI;

// Run via -batchmode -executeMethod RobotVideoValidation.Run -quit.
public static class RobotVideoValidation
{
    private static void Check(bool condition, string message)
    {
        if (!condition) throw new Exception("Robot video validation failed: " + message);
    }

    public static void Run()
    {
        var clock = new RobotVideoTiming();
        clock.Begin(100);
        Check(!clock.HasFrame && !clock.TimedOut(114.9, 15, 3), "normal view while connecting");
        Check(clock.TimedOut(115, 15, 3), "first-frame deadline");
        clock.FrameDecoded(116);
        Check(clock.HasFrame && !clock.TimedOut(118.9, 15, 3), "fresh video");
        Check(clock.TimedOut(119, 15, 3), "stalled media with live signaling");
        clock.Begin(120);
        Check(!clock.HasFrame && !clock.TimedOut(120, 15, 3), "reconnect resets old-frame state");

        EditorSceneManager.OpenScene("Assets/Scenes/Scene.unity");
        var manager = UnityEngine.Object.FindFirstObjectByType<VideoStreamManager>();
        Check(manager != null, "video manager scene binding");
        var serialized = new SerializedObject(manager);
        foreach (var name in new[] { "videoReceiver", "panelRenderer" })
            Check(serialized.FindProperty(name).objectReferenceValue != null, name + " scene binding");
        Check(serialized.FindProperty("automaticRobotVision").boolValue, "automatic mode enabled");
        var renderer = (VideoPanelRenderer)serialized.FindProperty("panelRenderer").objectReferenceValue;
        var panel = new SerializedObject(renderer);
        Check(panel.FindProperty("panelRoot").objectReferenceValue != null, "panel root binding");
        Check(panel.FindProperty("targetRawImage").objectReferenceValue != null, "video image binding");

        // Isolated scene verifies exact camera restoration and aspect handling.
        EditorSceneManager.NewScene(NewSceneSetup.EmptyScene, NewSceneMode.Single);
        var cameraObject = new GameObject("TestEye", typeof(Camera));
        cameraObject.tag = "MainCamera";
        var camera = cameraObject.GetComponent<Camera>();
        camera.clearFlags = CameraClearFlags.Depth;
        camera.backgroundColor = Color.magenta;
        var root = new GameObject("Video", typeof(RectTransform), typeof(Canvas));
        var imageObject = new GameObject("Image", typeof(RectTransform), typeof(RawImage));
        imageObject.transform.SetParent(root.transform, false);
        var system = new GameObject("System");
        renderer = system.AddComponent<VideoPanelRenderer>();
        typeof(VideoPanelRenderer).GetField("panelRoot", BindingFlags.Instance | BindingFlags.NonPublic).SetValue(renderer, root);
        typeof(VideoPanelRenderer).GetField("targetRawImage", BindingFlags.Instance | BindingFlags.NonPublic).SetValue(renderer, imageObject.GetComponent<RawImage>());
        renderer.SetRobotVisionActive(true);
        Check(!root.activeSelf && camera.backgroundColor == Color.magenta, "no texture must not hide original view");
        var texture = new Texture2D(1280, 720);
        renderer.SetTexture(texture);
        for (int i = 0; i < 3; i++)
        {
            renderer.SetRobotVisionActive(true);
            Check(root.activeSelf && camera.backgroundColor == Color.black, "robot mode activates");
            Check(root.transform.parent == camera.transform, "head-locked video");
            Check(imageObject.GetComponent<RawImage>().color == Color.white, "opaque untinted video");
            var size = root.GetComponent<RectTransform>().sizeDelta;
            Check(Mathf.Abs(size.x / size.y - 16f / 9) < 0.001f, "aspect ratio");
            renderer.SetRobotVisionActive(false);
            Check(!root.activeSelf && camera.backgroundColor == Color.magenta && camera.clearFlags == CameraClearFlags.Depth,
                  "original camera restored after disconnect/reconnect");
        }
        renderer.SetRobotVisionActive(true);
        // Simulate component teardown in edit mode explicitly.
        typeof(VideoPanelRenderer).GetMethod("OnDisable", BindingFlags.Instance | BindingFlags.NonPublic).Invoke(renderer, null);
        Check(camera.backgroundColor == Color.magenta && !root.activeSelf, "disable restores view");
        UnityEngine.Object.DestroyImmediate(texture);
        EditorSceneManager.NewScene(NewSceneSetup.EmptyScene, NewSceneMode.Single);
        Debug.Log("ROBOT_VIDEO_VALIDATION_PASSED");
    }
}
#endif
