package org.firstinspires.ftc.teamcode

import android.database.sqlite.SQLiteDatabase

/**
 * [Sql] on top of Android's own SQLite — the Robot Controller's native
 * database engine. No extra dependency ships on the robot, and the file it
 * writes is a plain SQLite file that any computer (or Python) can open.
 *
 * Android's SQLite and JDBC disagree about a few things (blob binding,
 * generated ids, how a row is read back), so this class smooths those over
 * and hands the shared code one consistent surface.
 */
class AndroidSql(private val db: SQLiteDatabase) : Sql {

    override fun update(sql: String, args: List<Any?>) {
        db.execSQL(sql, args.toTypedArray())
    }

    override fun query(sql: String, args: List<Any?>): List<Map<String, Any?>> {
        val out = mutableListOf<Map<String, Any?>>()
        // rawQuery binds its arguments as text; numbers and blobs are never
        // used as query arguments, so the text form is lossless here.
        val selection = args.map { arg ->
            when (arg) {
                null -> null
                is ByteArray -> throw IllegalArgumentException("blobs cannot be query arguments")
                else -> arg.toString()
            }
        }.toTypedArray()
        db.rawQuery(sql, selection).use { c ->
            while (c.moveToNext()) {
                val row = LinkedHashMap<String, Any?>()
                for (i in 0 until c.columnCount) {
                    row[c.getColumnName(i)] = when (c.getType(i)) {
                        android.database.Cursor.FIELD_TYPE_INTEGER -> c.getLong(i)
                        android.database.Cursor.FIELD_TYPE_FLOAT -> c.getDouble(i)
                        android.database.Cursor.FIELD_TYPE_STRING -> c.getString(i)
                        android.database.Cursor.FIELD_TYPE_BLOB -> c.getBlob(i)
                        else -> null
                    }
                }
                out.add(row)
            }
        }
        return out
    }

    override fun insertReturningId(sql: String, args: List<Any?>): Long {
        db.execSQL(sql, args.toTypedArray())
        db.rawQuery("SELECT last_insert_rowid()", null).use { c ->
            if (c.moveToFirst()) return c.getLong(0)
        }
        return -1L
    }
}
