package org.firstinspires.ftc.teamcode

import org.junit.Assert.assertEquals
import org.junit.Assert.assertTrue
import org.junit.Test
import java.io.File
import java.sql.DriverManager

/**
 * The Black Box session format is read by two independent implementations:
 * the robot writes it with SQLite from Android, and the Python tools in
 * `tools/` read it with the stdlib `sqlite3` module. If the two schemas
 * drift apart, every tool quietly starts returning nothing.
 *
 * These tests pin the schema down from both sides.
 */
class SessionSchemaTest {

    /** The DDL the robot uses, as one blob of SQL. */
    private fun kotlinDdl(): String = SessionSchema.DDL.joinToString("\n")

    /**
     * Pulls the DDL out of the Python fixture generator so the contract can
     * be checked without running Python.
     */
    private fun pythonDdl(): String {
        val file = locateFixtureScript()
        val text = file.readText()
        val start = text.indexOf("DDL = [")
        val end = text.indexOf("INDEXES = [")
        assertTrue("could not find the DDL list in ${file.path}", start >= 0 && end > start)
        return text.substring(start, end) + text.substring(end, text.indexOf("BASE_MS"))
    }

    private fun locateFixtureScript(): File {
        var dir: File? = File("").absoluteFile
        repeat(5) {
            val candidate = File(dir, "ci_scripts/generate_fixtures.py")
            if (candidate.exists()) return candidate
            dir = dir?.parentFile
        }
        throw AssertionError("ci_scripts/generate_fixtures.py not found above ${File("").absolutePath}")
    }

    /** table name -> set of column names, for every CREATE TABLE in the SQL. */
    private fun tablesOf(ddl: String): Map<String, Set<String>> {
        val result = mutableMapOf<String, Set<String>>()
        val matcher = Regex(
            "CREATE\\s+TABLE\\s+IF\\s+NOT\\s+EXISTS\\s+(\\w+)\\s*\\((.*?)\\n\\s*\\)",
            setOf(RegexOption.IGNORE_CASE, RegexOption.DOT_MATCHES_ALL),
        )
        for (m in matcher.findAll(ddl)) {
            val columns = mutableSetOf<String>()
            for (line in m.groupValues[2].lines()) {
                val trimmed = line.trim()
                if (trimmed.isEmpty() || trimmed.startsWith("--")) continue
                val word = trimmed.split(Regex("\\s+")).first().uppercase()
                if (word in setOf("PRIMARY", "UNIQUE", "FOREIGN", "CHECK", "CONSTRAINT")) continue
                columns.add(word.lowercase())
            }
            result[m.groupValues[1].lowercase()] = columns
        }
        return result
    }

    @Test
    fun `the python fixture writes the same tables and columns as the robot`() {
        val kotlin = tablesOf(kotlinDdl())
        val python = tablesOf(pythonDdl())

        assertEquals(
            "the Python tools and the robot must agree on the table set",
            kotlin.keys.sorted(),
            python.keys.sorted(),
        )
        for ((table, columns) in kotlin) {
            assertEquals(
                "column mismatch in table '$table'",
                columns.sorted(),
                python[table]!!.sorted(),
            )
        }
    }

    @Test
    fun `the schema creates a usable database`() {
        val file = File.createTempFile("session-schema", ".db")
        file.delete()
        try {
            DriverManager.getConnection("jdbc:sqlite:${file.absolutePath}").use { conn ->
                SessionSchema.create(JdbcSql(conn))
                conn.createStatement().use { st ->
                    st.executeUpdate(
                        "INSERT INTO session (session_id, label) VALUES ('s1', 'auto')",
                    )
                }
            }
            DriverManager.getConnection("jdbc:sqlite:${file.absolutePath}").use { conn ->
                conn.createStatement().use { st ->
                    val rs = st.executeQuery("SELECT session_id, label FROM session")
                    assertTrue(rs.next())
                    assertEquals("s1", rs.getString(1))
                    assertEquals("auto", rs.getString(2))
                }
            }
        } finally {
            file.delete()
        }
    }
}
