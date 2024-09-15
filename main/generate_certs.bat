@echo off
REM Update this to point to your correct OpenSSL executable and configuration file
set OPENSSL_EXE=C:\msys64\mingw64\bin\openssl.exe
set OPENSSL_CONF=C:\msys64\mingw64\ssl\openssl.cnf
set PASSWORD=yourpassword
set CERT_DIR=%~dp0  REM Directory where the script is located (current directory)

REM Create necessary directories, ignoring if they already exist
mkdir certs 2>nul

REM Step 1: Generate Root CA Private Key and Certificate
echo Generating Root CA private key...
%OPENSSL_EXE% genpkey -algorithm RSA -out certs/root_ca.key -aes256 -pass pass:%PASSWORD%
if %errorlevel% neq 0 (
    echo Failed to generate Root CA private key
    exit /b 1
)

echo Generating Root CA certificate...
%OPENSSL_EXE% req -config %OPENSSL_CONF% -x509 -new -key certs/root_ca.key -sha256 -days 3650 -out certs/root_ca.crt -passin pass:%PASSWORD% -subj "/C=US/ST=State/L=City/O=Organization/OU=OrgUnit/CN=RootCA"
if %errorlevel% neq 0 (
    echo Failed to generate Root CA certificate
    exit /b 1
)

REM Step 2: Generate Server Private Key and Certificate
echo Generating Server private key...
%OPENSSL_EXE% genpkey -algorithm RSA -out certs/server.key
if %errorlevel% neq 0 (
    echo Failed to generate Server private key
    exit /b 1
)

echo Generating Server CSR (Certificate Signing Request)...
%OPENSSL_EXE% req -config %OPENSSL_CONF% -new -key certs/server.key -out certs/server.csr -subj "/C=US/ST=State/L=City/O=Organization/OU=OrgUnit/CN=Server"
if %errorlevel% neq 0 (
    echo Failed to generate Server CSR
    exit /b 1
)

echo Signing Server certificate with Root CA...
%OPENSSL_EXE% x509 -req -in certs/server.csr -CA certs/root_ca.crt -CAkey certs/root_ca.key -CAcreateserial -out certs/server.crt -days 365 -sha256 -passin pass:%PASSWORD%
if %errorlevel% neq 0 (
    echo Failed to sign Server certificate
    exit /b 1
)

REM Step 3: (Optional) Generate Client Private Key and Certificate for mTLS
echo Generating Client private key...
%OPENSSL_EXE% genpkey -algorithm RSA -out certs/client.key
if %errorlevel% neq 0 (
    echo Failed to generate Client private key
    exit /b 1
)

echo Generating Client CSR...
%OPENSSL_EXE% req -config %OPENSSL_CONF% -new -key certs/client.key -out certs/client.csr -subj "/C=US/ST=State/L=City/O=Organization/OU=OrgUnit/CN=Client"
if %errorlevel% neq 0 (
    echo Failed to generate Client CSR
    exit /b 1
)

echo Signing Client certificate with Root CA...
%OPENSSL_EXE% x509 -req -in certs/client.csr -CA certs/root_ca.crt -CAkey certs/root_ca.key -CAcreateserial -out certs/client.crt -days 365 -sha256 -passin pass:%PASSWORD%
if %errorlevel% neq 0 (
    echo Failed to sign Client certificate
    exit /b 1
)

REM Step 4: Create CA bundle (needed for the Python script to verify server)
echo Creating CA bundle...
copy certs\root_ca.crt certs\ca_bundle.pem
if %errorlevel% neq 0 (
    echo Failed to create CA bundle
    exit /b 1
)

echo Certificate generation complete. Files are located in the 'certs' directory.
pause
