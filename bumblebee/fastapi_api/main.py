from fastapi import FastAPI, Request, HTTPException
import mysql.connector
import uvicorn
import json
import os
from UtilBLB import *
from fastapi.responses import FileResponse
app = FastAPI()
SP_META_FILE = "sp.txt"

BASE_DIR = Path(__file__).resolve().parent
FAVICON_PATH = BASE_DIR / "static" / "favicon.ico"
@app.get("/favicon.ico")
async def favicon():
    return FileResponse(FAVICON_PATH)

def get_connection(hostIP="172.30.1.4"):
    if isRealMachine:
        hostIP = "172.30.1.151"
    return mysql.connector.connect(
        host=hostIP,
        user="tact",
        password="bconnect@123",
        database="bconnectpoc"
    )

def load_or_fetch_metadata():
    if os.path.exists(SP_META_FILE):
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
        raise HTTPException(status_code=400, detail="Missing stored procedure name (?sp=...)")

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
    uvicorn.run("main:app", host="0.0.0.0", port=6003, reload=False)
