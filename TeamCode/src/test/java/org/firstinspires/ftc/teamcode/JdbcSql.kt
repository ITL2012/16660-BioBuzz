package org.firstinspires.ftc.teamcode

import java.sql.Connection
import java.sql.PreparedStatement
import java.sql.Types

/**
 * [Sql] on top of JDBC — the desktop half of the Black Box. It talks to
 * SQLite through the xerial driver, so a file recorded on the Robot
 * Controller opens here unchanged, and the Python tools read the same file
 * with their stdlib sqlite3.
 */
class JdbcSql(private val conn: Connection) : Sql {

    override fun update(sql: String, args: List<Any?>) {
        conn.prepareStatement(sql).use { ps ->
            bind(ps, args)
            ps.executeUpdate()
        }
    }

    override fun query(sql: String, args: List<Any?>): List<Map<String, Any?>> {
        val out = mutableListOf<Map<String, Any?>>()
        conn.prepareStatement(sql).use { ps ->
            bind(ps, args)
            ps.executeQuery().use { rs ->
                val meta = rs.metaData
                while (rs.next()) {
                    val row = LinkedHashMap<String, Any?>()
                    for (i in 1..meta.columnCount) {
                        row[meta.getColumnLabel(i)] = rs.getObject(i)
                    }
                    out.add(row)
                }
            }
        }
        return out
    }

    override fun insertReturningId(sql: String, args: List<Any?>): Long {
        conn.prepareStatement(sql).use { ps ->
            bind(ps, args)
            ps.executeUpdate()
        }
        conn.createStatement().use { st ->
            st.executeQuery("SELECT last_insert_rowid()").use { rs ->
                if (rs.next()) return rs.getLong(1)
            }
        }
        return -1L
    }

    private fun bind(ps: PreparedStatement, args: List<Any?>) {
        for ((i, arg) in args.withIndex()) {
            val index = i + 1
            when (arg) {
                null -> ps.setNull(index, Types.NULL)
                is ByteArray -> ps.setBytes(index, arg)
                is String -> ps.setString(index, arg)
                is Int -> ps.setInt(index, arg)
                is Long -> ps.setLong(index, arg)
                is Double -> ps.setDouble(index, arg)
                is Float -> ps.setFloat(index, arg)
                is Boolean -> ps.setInt(index, if (arg) 1 else 0)
                is Number -> ps.setDouble(index, arg.toDouble())
                else -> ps.setString(index, arg.toString())
            }
        }
    }
}
