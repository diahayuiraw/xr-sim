# Generated by the gRPC Python protocol compiler plugin. DO NOT EDIT!
"""Client and server classes corresponding to protobuf-defined services."""
import grpc
import warnings

from . import component_metadata_server_pb2 as component__metadata__server_dot_component__metadata__server__pb2

GRPC_GENERATED_VERSION = '1.63.0'
GRPC_VERSION = grpc.__version__
EXPECTED_ERROR_RELEASE = '1.65.0'
SCHEDULED_RELEASE_DATE = 'June 25, 2024'
_version_not_supported = False

try:
    from grpc._utilities import first_version_is_lower
    _version_not_supported = first_version_is_lower(GRPC_VERSION, GRPC_GENERATED_VERSION)
except ImportError:
    _version_not_supported = True

if _version_not_supported:
    warnings.warn(
        f'The grpc package installed is at version {GRPC_VERSION},'
        + f' but the generated code in component_metadata_server/component_metadata_server_pb2_grpc.py depends on'
        + f' grpcio>={GRPC_GENERATED_VERSION}.'
        + f' Please upgrade your grpc module to grpcio>={GRPC_GENERATED_VERSION}'
        + f' or downgrade your generated code using grpcio-tools<={GRPC_VERSION}.'
        + f' This warning will become an error in {EXPECTED_ERROR_RELEASE},'
        + f' scheduled for release on {SCHEDULED_RELEASE_DATE}.',
        RuntimeWarning
    )


class ComponentMetadataServerServiceStub(object):
    """Provide component metadata json definitions, such as parameters.
    """

    def __init__(self, channel):
        """Constructor.

        Args:
            channel: A grpc.Channel.
        """
        self.SetMetadata = channel.unary_unary(
                '/mavsdk.rpc.component_metadata_server.ComponentMetadataServerService/SetMetadata',
                request_serializer=component__metadata__server_dot_component__metadata__server__pb2.SetMetadataRequest.SerializeToString,
                response_deserializer=component__metadata__server_dot_component__metadata__server__pb2.SetMetadataResponse.FromString,
                _registered_method=True)


class ComponentMetadataServerServiceServicer(object):
    """Provide component metadata json definitions, such as parameters.
    """

    def SetMetadata(self, request, context):
        """
        Provide metadata (can only be called once)
        """
        context.set_code(grpc.StatusCode.UNIMPLEMENTED)
        context.set_details('Method not implemented!')
        raise NotImplementedError('Method not implemented!')


def add_ComponentMetadataServerServiceServicer_to_server(servicer, server):
    rpc_method_handlers = {
            'SetMetadata': grpc.unary_unary_rpc_method_handler(
                    servicer.SetMetadata,
                    request_deserializer=component__metadata__server_dot_component__metadata__server__pb2.SetMetadataRequest.FromString,
                    response_serializer=component__metadata__server_dot_component__metadata__server__pb2.SetMetadataResponse.SerializeToString,
            ),
    }
    generic_handler = grpc.method_handlers_generic_handler(
            'mavsdk.rpc.component_metadata_server.ComponentMetadataServerService', rpc_method_handlers)
    server.add_generic_rpc_handlers((generic_handler,))


 # This class is part of an EXPERIMENTAL API.
class ComponentMetadataServerService(object):
    """Provide component metadata json definitions, such as parameters.
    """

    @staticmethod
    def SetMetadata(request,
            target,
            options=(),
            channel_credentials=None,
            call_credentials=None,
            insecure=False,
            compression=None,
            wait_for_ready=None,
            timeout=None,
            metadata=None):
        return grpc.experimental.unary_unary(
            request,
            target,
            '/mavsdk.rpc.component_metadata_server.ComponentMetadataServerService/SetMetadata',
            component__metadata__server_dot_component__metadata__server__pb2.SetMetadataRequest.SerializeToString,
            component__metadata__server_dot_component__metadata__server__pb2.SetMetadataResponse.FromString,
            options,
            channel_credentials,
            insecure,
            call_credentials,
            compression,
            wait_for_ready,
            timeout,
            metadata,
            _registered_method=True)
