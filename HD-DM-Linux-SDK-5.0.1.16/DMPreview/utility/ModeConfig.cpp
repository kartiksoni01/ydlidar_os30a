#include "ModeConfig.h"
#include "sqlite3.h"
#include <sstream>

#define DEVICE_TABLE_COLUMN             4
#define DEVICE_TABLE_IDX_PID            1
#define DEVICE_TABLE_IDX_TABLE_NAME     2
#define DEVICE_TABLE_IDX_IMU_TYPE       3

#define PID_TABLE_COLUMN                12
#define PID_TABLE_IDX_MODE              0
#define PID_TABLE_IDX_MODE_DESC         1
#define PID_TABLE_IDX_L_RESOLUTION      2
#define PID_TABLE_IDX_D_RESOLUTION      3
#define PID_TABLE_IDX_K_RESOLUTION      4
#define PID_TABLE_IDX_T_RESOLUTION      5
#define PID_TABLE_IDX_DEPTH_TYPE        6
#define PID_TABLE_IDX_COLOR_FPS         7
#define PID_TABLE_IDX_DEPTH_FPS         8
#define PID_TABLE_IDX_USB_TYPE          9
#define PID_TABLE_IDX_RECTIFY_MODE      10
#define PID_TABLE_IDX_INTER_LEAVE_MODE  11

ModeConfig& g_ModeConfig = ModeConfig::GetModeConfig();

ModeConfig ModeConfig::m_ModeConfig;

ModeConfig::ModeConfig() : m_sq3( nullptr )
{
    ReadModeConfig();
}

ModeConfig::~ModeConfig()
{
    if ( m_sq3 ) sqlite3_close( m_sq3 );
}

const std::vector< ModeConfig::MODE_CONFIG >& ModeConfig::GetModeConfigList( const int iPID )
{
    static std::vector< ModeConfig::MODE_CONFIG > empty;

    if ( m_mapDeviceTable.find( iPID ) != m_mapDeviceTable.end() )
    {
        return m_mapDeviceTable[ iPID ].vecModeConfig;
    }
    return empty;
}

ModeConfig::IMU_TYPE ModeConfig::GetIMU_Type( const int iPID )
{
    if ( m_mapDeviceTable.find( iPID ) != m_mapDeviceTable.end() )
    {
        return m_mapDeviceTable[ iPID ].IMU_Type;
    }
    return IMU_NONE;
}

void ModeConfig::ReadModeConfig()
{
    if ( SQLITE_OK != sqlite3_open_v2( "../cfg/ModeConfig.db", &m_sq3, SQLITE_OPEN_READONLY, NULL ) )
    {
        return;
    }
    sqlite3_stmt *stmt = nullptr;

    int PID = 0;

    char szDecodeType[ 16 ] = { 0 };

    if ( SQLITE_OK != sqlite3_prepare_v2( m_sq3, "Select * from [DeviceTable]", EOF, &stmt, NULL ) )
    {
        return;
    }
    while ( SQLITE_ROW == sqlite3_step( stmt ) )
    {
        if ( DEVICE_TABLE_COLUMN != sqlite3_column_count( stmt ) )
        {
            continue;
        }
        if ( SQLITE_TEXT == sqlite3_column_type( stmt, DEVICE_TABLE_IDX_PID ) )
        {
            const char * valChar = ( char* )sqlite3_column_text(stmt, DEVICE_TABLE_IDX_PID );

            PID = valChar ? strtoul( valChar, NULL, 16 ) : 0;
        }
        if ( SQLITE_TEXT == sqlite3_column_type( stmt, DEVICE_TABLE_IDX_TABLE_NAME ) )
        {
            m_mapDeviceTable[ PID ].csTableName = QString::fromLocal8Bit( ( char* )sqlite3_column_text(stmt, DEVICE_TABLE_IDX_TABLE_NAME ));
        }
        if ( SQLITE_INTEGER == sqlite3_column_type( stmt, DEVICE_TABLE_IDX_IMU_TYPE ) )
        {
            m_mapDeviceTable[ PID ].IMU_Type = ( IMU_TYPE )sqlite3_column_int(stmt, DEVICE_TABLE_IDX_IMU_TYPE );
        }
    }
    sqlite3_finalize( stmt );

    QString csCommand;

    auto ResolutionParse = [ & ] ( const int iSqlIndex, MODE_CONFIG::RESOLUTION& xResolution, MODE_CONFIG::DECODE_TYPE& DecodeType )
    {
        sscanf( ( char* )sqlite3_column_text( stmt, iSqlIndex ), "%dx%d_%s", &xResolution.Width, &xResolution.Height, szDecodeType );

        if ( !strcmp( "MJPEG", szDecodeType ) ) DecodeType = MODE_CONFIG::MJPEG;
    };
    for ( auto& Table : m_mapDeviceTable )
    {
        csCommand.sprintf( "Select * from [%s]", Table.second.csTableName.toLocal8Bit().data() );

        if ( SQLITE_OK != sqlite3_prepare_v2( m_sq3, csCommand.toLocal8Bit().data(), EOF, &stmt, NULL ) )
        {
            continue;
        }
        while ( SQLITE_ROW == sqlite3_step( stmt ) )
        {
            if ( PID_TABLE_COLUMN != sqlite3_column_count( stmt ) )
            {
                continue;
            }
            MODE_CONFIG xModeConfig;

            if ( SQLITE_INTEGER == sqlite3_column_type( stmt, PID_TABLE_IDX_MODE ) )
            {
                xModeConfig.iMode = sqlite3_column_int(stmt, PID_TABLE_IDX_MODE );
            }
            if ( SQLITE_TEXT == sqlite3_column_type( stmt, PID_TABLE_IDX_MODE_DESC ) )
            {
                xModeConfig.csModeDesc = ( char* )sqlite3_column_text( stmt, PID_TABLE_IDX_MODE_DESC );
            }
            if ( SQLITE_TEXT == sqlite3_column_type( stmt, PID_TABLE_IDX_L_RESOLUTION ) )
            {
                ResolutionParse( PID_TABLE_IDX_L_RESOLUTION, xModeConfig.L_Resolution, xModeConfig.eDecodeType_L );
            }
            if ( SQLITE_TEXT == sqlite3_column_type( stmt, PID_TABLE_IDX_D_RESOLUTION ) )
            {
                sscanf( ( char* )sqlite3_column_text( stmt, PID_TABLE_IDX_D_RESOLUTION ), "%dx%d", &xModeConfig.D_Resolution.Width, &xModeConfig.D_Resolution.Height );
            }
            if ( SQLITE_TEXT == sqlite3_column_type( stmt, PID_TABLE_IDX_K_RESOLUTION ) )
            {
                ResolutionParse( PID_TABLE_IDX_K_RESOLUTION, xModeConfig.K_Resolution, xModeConfig.eDecodeType_K );
            }
            if ( SQLITE_TEXT == sqlite3_column_type( stmt, PID_TABLE_IDX_T_RESOLUTION ) )
            {
                ResolutionParse( PID_TABLE_IDX_T_RESOLUTION, xModeConfig.T_Resolution, xModeConfig.eDecodeType_T );
            }
            if ( SQLITE_TEXT == sqlite3_column_type( stmt, PID_TABLE_IDX_DEPTH_TYPE ) )
            {
                std::stringstream ssDepthType( ( char* )sqlite3_column_text( stmt, PID_TABLE_IDX_DEPTH_TYPE ) );

                for ( std::string each; std::getline( ssDepthType, each, ',' ); xModeConfig.vecDepthType.push_back( atoi( each.c_str() ) ) );
            }
            if ( SQLITE_TEXT == sqlite3_column_type( stmt, PID_TABLE_IDX_COLOR_FPS ) )
            {
                std::stringstream ssFrameRate( ( char* )sqlite3_column_text( stmt, PID_TABLE_IDX_COLOR_FPS ) );

                for ( std::string each; std::getline( ssFrameRate, each, ',' ); xModeConfig.vecColorFps.push_back( atoi( each.c_str() ) ) );
            }
            if ( SQLITE_TEXT == sqlite3_column_type( stmt, PID_TABLE_IDX_DEPTH_FPS ) )
            {
                std::stringstream ssFrameRate( ( char* )sqlite3_column_text( stmt, PID_TABLE_IDX_DEPTH_FPS ) );

                for ( std::string each; std::getline( ssFrameRate, each, ',' ); xModeConfig.vecDepthFps.push_back( atoi( each.c_str() ) ) );
            }
            if ( SQLITE_INTEGER == sqlite3_column_type( stmt, PID_TABLE_IDX_USB_TYPE ) )
            {
                xModeConfig.iUSB_Type = sqlite3_column_int(stmt, PID_TABLE_IDX_USB_TYPE );
            }
            if ( SQLITE_INTEGER == sqlite3_column_type( stmt, PID_TABLE_IDX_RECTIFY_MODE ) )
            {
                xModeConfig.bRectifyMode = ( sqlite3_column_int(stmt, PID_TABLE_IDX_RECTIFY_MODE ) > 0 );
            }
            if ( SQLITE_INTEGER == sqlite3_column_type( stmt, PID_TABLE_IDX_INTER_LEAVE_MODE ) )
            {
                xModeConfig.iInterLeaveModeFPS = sqlite3_column_int(stmt, PID_TABLE_IDX_INTER_LEAVE_MODE );
            }
            Table.second.vecModeConfig.push_back( std::move( xModeConfig ) );
        }
        sqlite3_finalize( stmt );
    }
}
