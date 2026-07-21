package com.smartmarketbot.hub.slam

import android.content.Context
import android.util.Log
import com.smartmarketbot.hub.navigation.NodeRegistry
import org.json.JSONArray
import org.json.JSONObject
import java.io.File

/**
 * MapPersistence — Phase 8: Save/Load occupancy grid map to local storage.
 *
 * Dùng JSON format (dễ debug, không cần thêm dependency).
 *
 * File structure:
 * {
 *   "version": 1,
 *   "resolution": 0.05,
 *   "width": 400,
 *   "height": 400,
 *   "grid": [int, int, int, ...]      // log-odds values, size = width * height
 *   "pose": {"x": 0.0, "y": 0.0, "theta": 0.0},
 *   "savedAt": "2026-07-21T01:00:00Z",
 *   "stats": {"updates": 1234, "occupiedCells": 5678}
 * }
 *
 * File location: context.filesDir/maps/<name>.json
 */
class MapPersistence(private val context: Context) {

    companion object {
        private const val TAG = "MapPersistence"
        private const val CURRENT_VERSION = 1
        private const val MAPS_DIR = "maps"
    }

    /**
     * Save map tới file internal storage.
     * @return absolute file path nếu thành công, null nếu lỗi.
     */
    fun saveMap(engine: SLAMEngine, name: String = "default", nodes: NodeRegistry? = null): String? {
        return try {
            val dir = File(context.filesDir, MAPS_DIR)
            if (!dir.exists()) dir.mkdirs()

            val file = File(dir, "$name.json")

            val grid = engine.getGrid()  // Float 2D array (probabilities)
            val pose = engine.getPose()
            val stats = engine.getStats()

            val root = JSONObject().apply {
                put("version", CURRENT_VERSION)
                put("resolution", 0.05)
                put("width", grid[0].size)
                put("height", grid.size)
                put("savedAt", System.currentTimeMillis())

                // Grid as flat int array of log-odds
                val (rawGrid, _, _) = engine.exportRawLogOdds()
                val gridJson = JSONArray()
                for (v in rawGrid) gridJson.put(v)
                put("grid", gridJson)

                put("pose", JSONObject().apply {
                    put("x", pose.x)
                    put("y", pose.y)
                    put("theta", pose.theta)
                })

                put("stats", JSONObject().apply {
                    put("updates", engine.totalUpdates)
                })

                // Phase 9 — line backbone nodes
                if (nodes != null) {
                    put("nodes", nodes.toJsonArray())
                } else {
                    put("nodes", JSONArray())
                }
            }

            file.writeText(root.toString())
            Log.i(TAG, "Saved map → ${file.absolutePath} (${grid.size}x${grid[0].size})")
            file.absolutePath
        } catch (e: Exception) {
            Log.e(TAG, "saveMap failed: ${e.message}")
            null
        }
    }

    /**
     * Load map từ file.
     * @return true nếu load thành công.
     */
    fun loadMap(engine: SLAMEngine, name: String = "default", nodes: NodeRegistry? = null): Boolean {
        return try {
            val file = File(File(context.filesDir, MAPS_DIR), "$name.json")
            if (!file.exists()) {
                Log.w(TAG, "Map file not found: ${file.absolutePath}")
                return false
            }

            val text = file.readText()
            val root = JSONObject(text)

            val version = root.optInt("version", 0)
            if (version != CURRENT_VERSION) {
                Log.w(TAG, "Map version mismatch: $version vs $CURRENT_VERSION")
            }

            val width = root.getInt("width")
            val height = root.getInt("height")
            val gridJson = root.getJSONArray("grid")

            val rawGrid = IntArray(width * height)
            for (i in 0 until gridJson.length()) {
                rawGrid[i] = gridJson.getInt(i)
            }

            val poseJson = root.optJSONObject("pose")
            engine.importRawLogOdds(rawGrid, width, height)

            if (poseJson != null) {
                engine.setPose(
                    poseJson.optDouble("x", 0.0).toFloat(),
                    poseJson.optDouble("y", 0.0).toFloat(),
                    poseJson.optDouble("theta", 0.0).toFloat()
                )
            }

            Log.i(TAG, "Loaded map ← ${file.absolutePath} (${height}x${width})")

            // Phase 9 — load nodes
            val nodesArr = root.optJSONArray("nodes")
            if (nodes != null && nodesArr != null) {
                nodes.loadFromJsonArray(nodesArr)
            }

            true
        } catch (e: Exception) {
            Log.e(TAG, "loadMap failed: ${e.message}")
            false
        }
    }

    /** Liệt kê các map files. */
    fun listMaps(): List<String> {
        val dir = File(context.filesDir, MAPS_DIR)
        if (!dir.exists()) return emptyList()
        return dir.listFiles { f -> f.extension == "json" }
            ?.map { it.nameWithoutExtension }
            ?.sorted()
            ?: emptyList()
    }

    /** Xóa map. */
    fun deleteMap(name: String): Boolean {
        val file = File(File(context.filesDir, MAPS_DIR), "$name.json")
        return file.exists() && file.delete()
    }
}