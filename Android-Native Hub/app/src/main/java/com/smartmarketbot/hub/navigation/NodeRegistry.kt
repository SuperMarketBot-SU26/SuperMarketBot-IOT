package com.smartmarketbot.hub.navigation

import android.util.Log
import org.json.JSONArray
import org.json.JSONObject

/**
 * NodeRegistry — Phase 9: ánh xạ nodeId ↔ (worldX, worldY, theta).
 *
 * Mỗi "dấu +" vật lý trên sàn tương ứng 1 node trên map.
 * Node ID được ESP32 gán thứ tự khi robot đi qua (lastNodeId++).
 *
 * Ứng dụng:
 *   - Khi ESP32 phát NodeEvent(nodeId), ta tra worldPose từ registry.
 *   - Nếu robot đang lệch (SLAM drift), snap pose về nodePose.
 *   - Line là backbone cứng → snap khi cross node là position fix tuyệt đối.
 *
 * Map editor flow (Phase 9.1):
 *   1. Robot đi dọc line, đếm node tự động qua sensor.
 *   2. SLAM đồng thời ước lượng (x, y) → registry lưu nodeId → slamPose.
 *   3. User có thể edit nodePose trên UI nếu SLAM sai.
 */
class NodeRegistry {

    data class Node(
        val id: Int,           // ID do ESP32 gán (1, 2, 3, ...)
        val x: Float,          // world X (m)
        val y: Float,          // world Y (m)
        val theta: Float = 0f, // heading tại node (rad)
        val label: String = "" // optional name: "Shelf A", "Counter"...
    ) {
        fun toJson(): JSONObject = JSONObject().apply {
            put("id", id)
            put("x", x.toDouble())
            put("y", y.toDouble())
            put("theta", theta.toDouble())
            put("label", label)
        }
        companion object {
            fun fromJson(j: JSONObject): Node = Node(
                id = j.optInt("id"),
                x = j.optDouble("x", 0.0).toFloat(),
                y = j.optDouble("y", 0.0).toFloat(),
                theta = j.optDouble("theta", 0.0).toFloat(),
                label = j.optString("label", "")
            )
        }
    }

    private val nodes = mutableMapOf<Int, Node>()
    private val nodeOrder = mutableListOf<Int>()  // giữ insertion order

    /** Lưu hoặc update node theo id. */
    fun upsert(node: Node) {
        if (node.id <= 0) return
        if (!nodes.containsKey(node.id)) nodeOrder.add(node.id)
        nodes[node.id] = node
        Log.i(TAG, "Upsert node #${node.id} → (${"%.2f".format(node.x)}, ${"%.2f".format(node.y)}) [${node.label}]")
    }

    /** Tạo node mới từ pose hiện tại của SLAM. Trả về nodeId. */
    fun addFromPose(nodeId: Int, x: Float, y: Float, theta: Float, label: String = ""): Node {
        val n = Node(nodeId, x, y, theta, label)
        upsert(n)
        return n
    }

    fun get(id: Int): Node? = nodes[id]

    fun remove(id: Int): Boolean {
        val ok = nodes.remove(id) != null
        if (ok) nodeOrder.remove(id)
        return ok
    }

    fun size(): Int = nodes.size
    fun all(): List<Node> = nodeOrder.mapNotNull { nodes[it] }
    fun ids(): List<Int> = nodeOrder.toList()

    /**
     * Tìm node gần nhất với pose (x, y) trong bán kính maxDistM.
     * Dùng khi SLAM drift → snap về node gần nhất để tăng accuracy.
     */
    fun nearestWithin(x: Float, y: Float, maxDistM: Float = 0.5f): Node? {
        var best: Node? = null
        var bestDist2 = maxDistM * maxDistM
        for (n in nodes.values) {
            val dx = n.x - x
            val dy = n.y - y
            val d2 = dx * dx + dy * dy
            if (d2 < bestDist2) {
                bestDist2 = d2
                best = n
            }
        }
        return best
    }

    /** Distance từ (x, y) tới node tiếp theo (theo thứ tự id). */
    fun distToNext(x: Float, y: Float, afterId: Int = -1): Pair<Node, Float>? {
        var best: Node? = null
        var bestDist = Float.MAX_VALUE
        for (id in nodeOrder) {
            if (id <= afterId) continue
            val n = nodes[id] ?: continue
            val dx = n.x - x
            val dy = n.y - y
            val d = kotlin.math.sqrt(dx * dx + dy * dy)
            if (d < bestDist) {
                bestDist = d
                best = n
            }
        }
        return best?.let { it to bestDist }
    }

    /** Serialize to JSON array. */
    fun toJsonArray(): JSONArray {
        val arr = JSONArray()
        for (id in nodeOrder) {
            val n = nodes[id] ?: continue
            arr.put(n.toJson())
        }
        return arr
    }

    /** Load từ JSON array (do MapPersistence). */
    fun loadFromJsonArray(arr: JSONArray) {
        nodes.clear()
        nodeOrder.clear()
        for (i in 0 until arr.length()) {
            val n = Node.fromJson(arr.getJSONObject(i))
            upsert(n)
        }
        Log.i(TAG, "Loaded ${nodes.size} nodes from JSON")
    }

    fun clear() {
        nodes.clear()
        nodeOrder.clear()
    }

    companion object { private const val TAG = "NodeRegistry" }
}
