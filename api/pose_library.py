import os
import json
import base64
from aiohttp import web

# ===== 固定パス =====
POSE_LIBRARY_PATH = "/notebooks/ComfyUI/custom_nodes/ComfyUI-VNCCS-Utils/PoseLibrary"

def get_library_path():
    """Returns fixed PoseLibrary path (Paperspace stable)."""
    os.makedirs(POSE_LIBRARY_PATH, exist_ok=True)
    return POSE_LIBRARY_PATH


# ===== 共通ログ（デバッグ用）=====
def log(msg):
    print(f"[PoseLibrary] {msg}")


# ===== 一覧取得 =====
async def list_poses(request):
    """GET /vnccs/pose_library/list"""
    full_details = request.query.get("full") == "true"
    lib_path = get_library_path()

    log(f"LIST from: {lib_path}")

    poses = []

    try:
        filenames = os.listdir(lib_path)
    except Exception as e:
        log(f"LIST ERROR: {e}")
        return web.json_response({"poses": []})

    for filename in filenames:
        if filename.endswith(".json"):
            name = filename[:-5]

            pose_path = os.path.join(lib_path, filename)
            preview_path = os.path.join(lib_path, f"{name}.png")

            has_preview = os.path.exists(preview_path)

            pose_data = None
            if full_details:
                try:
                    with open(pose_path, "r", encoding="utf-8") as f:
                        pose_data = json.load(f)
                except Exception as e:
                    pose_data = {"_error": str(e)}

            poses.append({
                "name": name,
                "has_preview": has_preview,
                "data": pose_data
            })

    return web.json_response({
        "poses": sorted(poses, key=lambda x: x["name"])
    })


# ===== 1件取得 =====
async def get_pose(request):
    """GET /vnccs/pose_library/get/{name}"""
    name = request.match_info.get("name")

    if not name:
        return web.json_response({"error": "Name required"}, status=400)

    lib_path = get_library_path()

    pose_path = os.path.join(lib_path, f"{name}.json")
    preview_path = os.path.join(lib_path, f"{name}.png")

    log(f"GET: {pose_path}")

    if not os.path.exists(pose_path):
        return web.json_response({"error": "Pose not found"}, status=404)

    try:
        with open(pose_path, "r", encoding="utf-8") as f:
            pose_data = json.load(f)
    except Exception as e:
        return web.json_response({"error": str(e)}, status=500)

    preview_b64 = None

    if os.path.exists(preview_path):
        try:
            with open(preview_path, "rb") as f:
                preview_b64 = base64.b64encode(f.read()).decode("utf-8")
        except Exception as e:
            log(f"Preview load error: {e}")

    return web.json_response({
        "name": name,
        "pose": pose_data,
        "preview": preview_b64
    })


# ===== 保存 =====
async def save_pose(request):
    """POST /vnccs/pose_library/save"""
    try:
        data = await request.json()
    except Exception:
        return web.json_response({"error": "Invalid JSON"}, status=400)

    name = data.get("name")
    pose = data.get("pose")
    preview_b64 = data.get("preview")

    if not name or pose is None:
        return web.json_response({"error": "Name and pose required"}, status=400)

    # ファイル名安全化
    name = "".join(c for c in name if c.isalnum() or c in "-_ ").strip()

    if not name:
        return web.json_response({"error": "Invalid name"}, status=400)

    lib_path = get_library_path()

    pose_path = os.path.join(lib_path, f"{name}.json")
    preview_path = os.path.join(lib_path, f"{name}.png")

    log(f"SAVE: {pose_path}")

    # JSON保存
    try:
        with open(pose_path, "w", encoding="utf-8") as f:
            json.dump(pose, f, indent=2, ensure_ascii=False)
    except Exception as e:
        return web.json_response({"error": str(e)}, status=500)

    # プレビュー保存
    if preview_b64:
        try:
            if "," in preview_b64:
                preview_b64 = preview_b64.split(",", 1)[1]

            with open(preview_path, "wb") as f:
                f.write(base64.b64decode(preview_b64))
        except Exception as e:
            log(f"Preview save error: {e}")

    return web.json_response({
        "success": True,
        "name": name,
        "path": pose_path
    })


# ===== 削除 =====
async def delete_pose(request):
    """DELETE /vnccs/pose_library/delete/{name}"""
    name = request.match_info.get("name")

    if not name:
        return web.json_response({"error": "Name required"}, status=400)

    lib_path = get_library_path()

    pose_path = os.path.join(lib_path, f"{name}.json")
    preview_path = os.path.join(lib_path, f"{name}.png")

    log(f"DELETE: {pose_path}")

    if not os.path.exists(pose_path):
        return web.json_response({"error": "Pose not found"}, status=404)

    try:
        os.remove(pose_path)
        if os.path.exists(preview_path):
            os.remove(preview_path)
    except Exception as e:
        return web.json_response({"error": str(e)}, status=500)

    return web.json_response({"success": True})


# ===== プレビュー取得 =====
async def get_preview(request):
    """GET /vnccs/pose_library/preview/{name}"""
    name = request.match_info.get("name")

    if not name:
        return web.Response(status=400)

    lib_path = get_library_path()
    preview_path = os.path.join(lib_path, f"{name}.png")

    if not os.path.exists(preview_path):
        return web.Response(status=404)

    try:
        with open(preview_path, "rb") as f:
            return web.Response(body=f.read(), content_type="image/png")
    except Exception:
        return web.Response(status=500)


# ===== 同期（そのまま）=====
async def upload_pose_sync(request):
    """POST /vnccs/pose_sync/upload_capture"""
    try:
        data = await request.json()
        node_id = data.get("node_id")

        if not node_id:
            return web.json_response({"error": "No node_id"}, status=400)

        import folder_paths
        temp_dir = folder_paths.get_temp_directory()

        filepath = os.path.join(temp_dir, f"vnccs_debug_{node_id}.json")

        with open(filepath, "w", encoding="utf-8") as f:
            json.dump(data, f, ensure_ascii=False)

        return web.json_response({"status": "ok"})

    except Exception as e:
        return web.json_response({"error": str(e)}, status=500)


# ===== ルート登録 =====
def register_routes(app):
    log("PoseLibrary API Loaded")

    app.router.add_get("/vnccs/pose_library/list", list_poses)
    app.router.add_get("/vnccs/pose_library/get/{name}", get_pose)
    app.router.add_post("/vnccs/pose_library/save", save_pose)
    app.router.add_delete("/vnccs/pose_library/delete/{name}", delete_pose)
    app.router.add_get("/vnccs/pose_library/preview/{name}", get_preview)

    app.router.add_post("/vnccs/pose_sync/upload_capture", upload_pose_sync)
    app.router.add_post("/vnccs/debug/upload_capture", upload_pose_sync)