
import json
import requests

SYSTEM_PROMPT = (
    "You are a robot navigation rule compiler. "
    "Given a 2D map summary (grid size, resolution, origin), optional landmarks, "
    "and natural-language rules, output ONLY one JSON object that concretizes "
    "each rule into map-anchored shapes: circles (center[x,y], radius), "
    "polygons (coordinates[[x,y],...]), or lines (start/end). "
    "Units are meters in the 'map' frame. Be conservative; if uncertain, "
    "return a smaller region or empty list."
)

def build_user_payload(grid_w, grid_h, resolution, origin_xy, landmarks, semantic_rules):
    return {
        "grid_size": [grid_w, grid_h],      # coarse map 요약값이 있으면 전달(없으면 생략 가능)
        "resolution": resolution,           # m/cell
        "origin": origin_xy,                # [ox, oy]
        "landmarks": landmarks or {},
        "semantic_rules": semantic_rules,
        "need_format": {
            "speed_zones": [{"id": "", "shape": {"type": "circle|polygon"}, "max_speed": 0.3}],
            "keepout_zones": [{"id": "", "shape": {"type": "circle|polygon"}}],
            "lane_preferences": [{"id": "", "line": {"start": [0,0], "end":[1,1]}, "preference": "right|left"}]
        }
    }

def call_ollama(url, model, system, user_json, timeout_s=12, max_tokens=512):
    body = {
        "model": model,
        "messages": [
            {"role": "system", "content": system},
            {"role": "user", "content": json.dumps(user_json, ensure_ascii=False)}
        ],
        "format": {"type":"json_object"},
        "stream": False,
        "options": {"num_predict": max_tokens}
    }
    r = requests.post(url, json=body, timeout=timeout_s)
    r.raise_for_status()
    content = r.json()["message"]["content"]
    return json.loads(content)

def compile_semantic_rules(og_meta, params_llm, landmarks, semantic_rules):
    """
    og_meta: dict with keys {width, height, resolution, origin_xy}
    params_llm: dict with keys {enable, url, model, timeout_s, max_tokens, confidence_threshold}
    return: dict {"speed_zones": [...], "keepout_zones": [...], "lane_preferences": [...]}
    """
    if not params_llm.get("enable", False) or not semantic_rules:
        return {"speed_zones": [], "keepout_zones": [], "lane_preferences": []}

    try:
        user = build_user_payload(
            grid_w=og_meta.get("width"),
            grid_h=og_meta.get("height"),
            resolution=og_meta.get("resolution"),
            origin_xy=og_meta.get("origin_xy"),
            landmarks=landmarks,
            semantic_rules=semantic_rules
        )
        out = call_ollama(
            url=params_llm["url"],
            model=params_llm["model"],
            system=SYSTEM_PROMPT,
            user_json=user,
            timeout_s=params_llm.get("timeout_s", 12),
            max_tokens=params_llm.get("max_tokens", 512),
        )
        # 방어적 파싱
        speed_zones = out.get("speed_zones", []) or []
        keepout_zones = out.get("keepout_zones", []) or []
        lane_prefs   = out.get("lane_preferences", []) or []
        return {
            "speed_zones": speed_zones,
            "keepout_zones": keepout_zones,
            "lane_preferences": lane_prefs
        }
    except Exception as e:
        # 실패 시 빈 결과 반환(상위에서 fallback 적용)
        print(f"[LLMResolver] compile failed: {e}")
        return {"speed_zones": [], "keepout_zones": [], "lane_preferences": []}
