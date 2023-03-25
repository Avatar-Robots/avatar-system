using UnityEngine;
using Vuforia;

public class ButtonEventsHandler : MonoBehaviour
{
    public GameObject AvatarGameObject;

    public ModelTargetBehaviour ModelTarget;

    public void OnExitButtonClicked()
    {
        Application.Quit();
    }

    public void toggleAvatarVisibility()
    {
        AvatarGameObject.SetActive(!AvatarGameObject.activeSelf);
    }

    public void teleportAvatarToTarget()
    {
        var modelTransform = ModelTarget.gameObject.transform;
        var avatarTransform = AvatarGameObject.transform;
        avatarTransform.SetPositionAndRotation(modelTransform.position,
            modelTransform.rotation * Quaternion.Euler(0, -90, -90));
        avatarTransform.position -= 0.14f * avatarTransform.right + 1.12f * avatarTransform.up -
                                    0.18f * avatarTransform.forward;
    }
}