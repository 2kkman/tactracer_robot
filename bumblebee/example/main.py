from fastapi import FastAPI, Request, HTTPException
import mysql.connector
import uvicorn
import json
import os
import pandas as pd

app = FastAPI()
SP_META_FILE = "sp.txt"

def get_connection():
    return mysql.connector.connect(
        host="172.30.1.4",
        user="tact",
        password="bconnect@123",
        database="bconnectpoc"
    )
    
def export_sample_rows(limit=10):
    conn = get_connection()
    cursor = conn.cursor()

    # 테이블 목록 조회
    cursor.execute("SHOW TABLES")
    tables = cursor.fetchall()

    for (table_name,) in tables:
        print(f"Exporting {table_name}.csv...")

        # 테이블 내용 조회
        query = f"SELECT * FROM `{table_name}` LIMIT {limit}"
        df = pd.read_sql(query, conn)

        # 현재 폴더에 저장
        filename = f"{table_name}.csv"
        df.to_csv(filename, index=False, encoding="utf-8-sig")  # Excel 호환 UTF-8

    cursor.close()
    conn.close()
    print("✅ Export completed.")
    
def read_meta():
    conn = get_connection()
    cursor = conn.cursor(dictionary=True)
    cursor.execute("""
        SELECT SPECIFIC_NAME AS procedure_name
        FROM information_schema.ROUTINES
        WHERE ROUTINE_TYPE='PROCEDURE' AND ROUTINE_SCHEMA=%s
    """, (conn.database,))
    procedures = cursor.fetchall()
    result = []
    for proc in procedures:
        name = proc["procedure_name"]
        cursor.execute("""
            SELECT PARAMETER_NAME, DATA_TYPE, DTD_IDENTIFIER, PARAMETER_MODE
            FROM information_schema.PARAMETERS
            WHERE SPECIFIC_NAME = %s AND SPECIFIC_SCHEMA = %s
        """, (name, conn.database))
        params = cursor.fetchall()
        result.append({
            "procedure_name": name,
            "parameters": params
        })
    cursor.close()
    conn.close()
    
    return result

def load_or_fetch_metadata():
    if os.path.exists(SP_META_FILE):
        export_sample_rows(50)
        with open(SP_META_FILE, "r", encoding="utf-8") as f:
            return json.load(f)
    else:
        conn = get_connection()
        cursor = conn.cursor(dictionary=True)
        cursor.execute("""
            SELECT SPECIFIC_NAME AS procedure_name
            FROM information_schema.ROUTINES
            WHERE ROUTINE_TYPE='PROCEDURE' AND ROUTINE_SCHEMA=%s
        """, (conn.database,))
        procedures = cursor.fetchall()

        result = []
        for proc in procedures:
            name = proc["procedure_name"]
            cursor.execute("""
                SELECT PARAMETER_NAME, DATA_TYPE, DTD_IDENTIFIER, PARAMETER_MODE
                FROM information_schema.PARAMETERS
                WHERE SPECIFIC_NAME = %s AND SPECIFIC_SCHEMA = %s
                ORDER BY ORDINAL_POSITION
            """, (name, conn.database))
            params = cursor.fetchall()
            result.append({
                "procedure_name": name,
                "parameters": params
            })

        cursor.close()
        conn.close()

        with open(SP_META_FILE, "w", encoding="utf-8") as f:
            json.dump(result, f, ensure_ascii=False, indent=2)
        return result

def get_procedure_parameters(sp_name):
    metadata = load_or_fetch_metadata()
    for proc in metadata:
        if proc["procedure_name"].lower() == sp_name.lower():
            return proc["parameters"]
    return []

@app.get("/")
async def call_stored_procedure(request: Request):
    query_params = dict(request.query_params)
    sp_name = query_params.pop("sp", None)

    if not sp_name:
        return load_or_fetch_metadata()

    param_meta = get_procedure_parameters(sp_name)
    if not param_meta:
        raise HTTPException(status_code=404, detail=f"Stored procedure '{sp_name}' not found.")

    expected_params = [p["PARAMETER_NAME"] for p in param_meta if p["PARAMETER_MODE"] == "IN"]
    passed_params = [query_params.get(name) for name in expected_params]

    if None in passed_params:
        missing = [name for name, value in zip(expected_params, passed_params) if value is None]
        raise HTTPException(status_code=400, detail=f"Missing required parameters: {missing}")

    placeholders = ", ".join(["%s"] * len(expected_params))
    call_query = f"CALL {sp_name}({placeholders})"

    conn = get_connection()
    cursor = conn.cursor(dictionary=True)
    cursor.execute(call_query, passed_params)
    results = cursor.fetchall()
    cursor.close()
    conn.close()

    return results

if __name__ == "__main__":
    uvicorn.run("main:app", host="0.0.0.0", port=6004, reload=False)
