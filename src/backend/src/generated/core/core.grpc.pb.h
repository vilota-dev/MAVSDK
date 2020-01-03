// Generated by the gRPC C++ plugin.
// If you make any local change, they will be lost.
// source: core/core.proto
#ifndef GRPC_core_2fcore_2eproto__INCLUDED
#define GRPC_core_2fcore_2eproto__INCLUDED

#include "core/core.pb.h"

#include <functional>
#include <grpcpp/impl/codegen/async_generic_service.h>
#include <grpcpp/impl/codegen/async_stream.h>
#include <grpcpp/impl/codegen/async_unary_call.h>
#include <grpcpp/impl/codegen/client_callback.h>
#include <grpcpp/impl/codegen/client_context.h>
#include <grpcpp/impl/codegen/completion_queue.h>
#include <grpcpp/impl/codegen/method_handler.h>
#include <grpcpp/impl/codegen/proto_utils.h>
#include <grpcpp/impl/codegen/rpc_method.h>
#include <grpcpp/impl/codegen/server_callback.h>
#include <grpcpp/impl/codegen/server_callback_handlers.h>
#include <grpcpp/impl/codegen/server_context.h>
#include <grpcpp/impl/codegen/service_type.h>
#include <grpcpp/impl/codegen/status.h>
#include <grpcpp/impl/codegen/stub_options.h>
#include <grpcpp/impl/codegen/sync_stream.h>

namespace grpc_impl {
class CompletionQueue;
class ServerCompletionQueue;
class ServerContext;
}  // namespace grpc_impl

namespace grpc {
namespace experimental {
template <typename RequestT, typename ResponseT>
class MessageAllocator;
}  // namespace experimental
}  // namespace grpc

namespace mavsdk {
namespace rpc {
namespace core {

// Access to the connection state and running plugins.
class CoreService final {
 public:
  static constexpr char const* service_full_name() {
    return "mavsdk.rpc.core.CoreService";
  }
  class StubInterface {
   public:
    virtual ~StubInterface() {}
    // Subscribe to 'connection state' updates.
    std::unique_ptr< ::grpc::ClientReaderInterface< ::mavsdk::rpc::core::ConnectionStateResponse>> SubscribeConnectionState(::grpc::ClientContext* context, const ::mavsdk::rpc::core::SubscribeConnectionStateRequest& request) {
      return std::unique_ptr< ::grpc::ClientReaderInterface< ::mavsdk::rpc::core::ConnectionStateResponse>>(SubscribeConnectionStateRaw(context, request));
    }
    std::unique_ptr< ::grpc::ClientAsyncReaderInterface< ::mavsdk::rpc::core::ConnectionStateResponse>> AsyncSubscribeConnectionState(::grpc::ClientContext* context, const ::mavsdk::rpc::core::SubscribeConnectionStateRequest& request, ::grpc::CompletionQueue* cq, void* tag) {
      return std::unique_ptr< ::grpc::ClientAsyncReaderInterface< ::mavsdk::rpc::core::ConnectionStateResponse>>(AsyncSubscribeConnectionStateRaw(context, request, cq, tag));
    }
    std::unique_ptr< ::grpc::ClientAsyncReaderInterface< ::mavsdk::rpc::core::ConnectionStateResponse>> PrepareAsyncSubscribeConnectionState(::grpc::ClientContext* context, const ::mavsdk::rpc::core::SubscribeConnectionStateRequest& request, ::grpc::CompletionQueue* cq) {
      return std::unique_ptr< ::grpc::ClientAsyncReaderInterface< ::mavsdk::rpc::core::ConnectionStateResponse>>(PrepareAsyncSubscribeConnectionStateRaw(context, request, cq));
    }
    // Get a list of currently running plugins.
    virtual ::grpc::Status ListRunningPlugins(::grpc::ClientContext* context, const ::mavsdk::rpc::core::ListRunningPluginsRequest& request, ::mavsdk::rpc::core::ListRunningPluginsResponse* response) = 0;
    std::unique_ptr< ::grpc::ClientAsyncResponseReaderInterface< ::mavsdk::rpc::core::ListRunningPluginsResponse>> AsyncListRunningPlugins(::grpc::ClientContext* context, const ::mavsdk::rpc::core::ListRunningPluginsRequest& request, ::grpc::CompletionQueue* cq) {
      return std::unique_ptr< ::grpc::ClientAsyncResponseReaderInterface< ::mavsdk::rpc::core::ListRunningPluginsResponse>>(AsyncListRunningPluginsRaw(context, request, cq));
    }
    std::unique_ptr< ::grpc::ClientAsyncResponseReaderInterface< ::mavsdk::rpc::core::ListRunningPluginsResponse>> PrepareAsyncListRunningPlugins(::grpc::ClientContext* context, const ::mavsdk::rpc::core::ListRunningPluginsRequest& request, ::grpc::CompletionQueue* cq) {
      return std::unique_ptr< ::grpc::ClientAsyncResponseReaderInterface< ::mavsdk::rpc::core::ListRunningPluginsResponse>>(PrepareAsyncListRunningPluginsRaw(context, request, cq));
    }
    class experimental_async_interface {
     public:
      virtual ~experimental_async_interface() {}
      // Subscribe to 'connection state' updates.
      virtual void SubscribeConnectionState(::grpc::ClientContext* context, ::mavsdk::rpc::core::SubscribeConnectionStateRequest* request, ::grpc::experimental::ClientReadReactor< ::mavsdk::rpc::core::ConnectionStateResponse>* reactor) = 0;
      // Get a list of currently running plugins.
      virtual void ListRunningPlugins(::grpc::ClientContext* context, const ::mavsdk::rpc::core::ListRunningPluginsRequest* request, ::mavsdk::rpc::core::ListRunningPluginsResponse* response, std::function<void(::grpc::Status)>) = 0;
      virtual void ListRunningPlugins(::grpc::ClientContext* context, const ::grpc::ByteBuffer* request, ::mavsdk::rpc::core::ListRunningPluginsResponse* response, std::function<void(::grpc::Status)>) = 0;
      virtual void ListRunningPlugins(::grpc::ClientContext* context, const ::mavsdk::rpc::core::ListRunningPluginsRequest* request, ::mavsdk::rpc::core::ListRunningPluginsResponse* response, ::grpc::experimental::ClientUnaryReactor* reactor) = 0;
      virtual void ListRunningPlugins(::grpc::ClientContext* context, const ::grpc::ByteBuffer* request, ::mavsdk::rpc::core::ListRunningPluginsResponse* response, ::grpc::experimental::ClientUnaryReactor* reactor) = 0;
    };
    virtual class experimental_async_interface* experimental_async() { return nullptr; }
  private:
    virtual ::grpc::ClientReaderInterface< ::mavsdk::rpc::core::ConnectionStateResponse>* SubscribeConnectionStateRaw(::grpc::ClientContext* context, const ::mavsdk::rpc::core::SubscribeConnectionStateRequest& request) = 0;
    virtual ::grpc::ClientAsyncReaderInterface< ::mavsdk::rpc::core::ConnectionStateResponse>* AsyncSubscribeConnectionStateRaw(::grpc::ClientContext* context, const ::mavsdk::rpc::core::SubscribeConnectionStateRequest& request, ::grpc::CompletionQueue* cq, void* tag) = 0;
    virtual ::grpc::ClientAsyncReaderInterface< ::mavsdk::rpc::core::ConnectionStateResponse>* PrepareAsyncSubscribeConnectionStateRaw(::grpc::ClientContext* context, const ::mavsdk::rpc::core::SubscribeConnectionStateRequest& request, ::grpc::CompletionQueue* cq) = 0;
    virtual ::grpc::ClientAsyncResponseReaderInterface< ::mavsdk::rpc::core::ListRunningPluginsResponse>* AsyncListRunningPluginsRaw(::grpc::ClientContext* context, const ::mavsdk::rpc::core::ListRunningPluginsRequest& request, ::grpc::CompletionQueue* cq) = 0;
    virtual ::grpc::ClientAsyncResponseReaderInterface< ::mavsdk::rpc::core::ListRunningPluginsResponse>* PrepareAsyncListRunningPluginsRaw(::grpc::ClientContext* context, const ::mavsdk::rpc::core::ListRunningPluginsRequest& request, ::grpc::CompletionQueue* cq) = 0;
  };
  class Stub final : public StubInterface {
   public:
    Stub(const std::shared_ptr< ::grpc::ChannelInterface>& channel);
    std::unique_ptr< ::grpc::ClientReader< ::mavsdk::rpc::core::ConnectionStateResponse>> SubscribeConnectionState(::grpc::ClientContext* context, const ::mavsdk::rpc::core::SubscribeConnectionStateRequest& request) {
      return std::unique_ptr< ::grpc::ClientReader< ::mavsdk::rpc::core::ConnectionStateResponse>>(SubscribeConnectionStateRaw(context, request));
    }
    std::unique_ptr< ::grpc::ClientAsyncReader< ::mavsdk::rpc::core::ConnectionStateResponse>> AsyncSubscribeConnectionState(::grpc::ClientContext* context, const ::mavsdk::rpc::core::SubscribeConnectionStateRequest& request, ::grpc::CompletionQueue* cq, void* tag) {
      return std::unique_ptr< ::grpc::ClientAsyncReader< ::mavsdk::rpc::core::ConnectionStateResponse>>(AsyncSubscribeConnectionStateRaw(context, request, cq, tag));
    }
    std::unique_ptr< ::grpc::ClientAsyncReader< ::mavsdk::rpc::core::ConnectionStateResponse>> PrepareAsyncSubscribeConnectionState(::grpc::ClientContext* context, const ::mavsdk::rpc::core::SubscribeConnectionStateRequest& request, ::grpc::CompletionQueue* cq) {
      return std::unique_ptr< ::grpc::ClientAsyncReader< ::mavsdk::rpc::core::ConnectionStateResponse>>(PrepareAsyncSubscribeConnectionStateRaw(context, request, cq));
    }
    ::grpc::Status ListRunningPlugins(::grpc::ClientContext* context, const ::mavsdk::rpc::core::ListRunningPluginsRequest& request, ::mavsdk::rpc::core::ListRunningPluginsResponse* response) override;
    std::unique_ptr< ::grpc::ClientAsyncResponseReader< ::mavsdk::rpc::core::ListRunningPluginsResponse>> AsyncListRunningPlugins(::grpc::ClientContext* context, const ::mavsdk::rpc::core::ListRunningPluginsRequest& request, ::grpc::CompletionQueue* cq) {
      return std::unique_ptr< ::grpc::ClientAsyncResponseReader< ::mavsdk::rpc::core::ListRunningPluginsResponse>>(AsyncListRunningPluginsRaw(context, request, cq));
    }
    std::unique_ptr< ::grpc::ClientAsyncResponseReader< ::mavsdk::rpc::core::ListRunningPluginsResponse>> PrepareAsyncListRunningPlugins(::grpc::ClientContext* context, const ::mavsdk::rpc::core::ListRunningPluginsRequest& request, ::grpc::CompletionQueue* cq) {
      return std::unique_ptr< ::grpc::ClientAsyncResponseReader< ::mavsdk::rpc::core::ListRunningPluginsResponse>>(PrepareAsyncListRunningPluginsRaw(context, request, cq));
    }
    class experimental_async final :
      public StubInterface::experimental_async_interface {
     public:
      void SubscribeConnectionState(::grpc::ClientContext* context, ::mavsdk::rpc::core::SubscribeConnectionStateRequest* request, ::grpc::experimental::ClientReadReactor< ::mavsdk::rpc::core::ConnectionStateResponse>* reactor) override;
      void ListRunningPlugins(::grpc::ClientContext* context, const ::mavsdk::rpc::core::ListRunningPluginsRequest* request, ::mavsdk::rpc::core::ListRunningPluginsResponse* response, std::function<void(::grpc::Status)>) override;
      void ListRunningPlugins(::grpc::ClientContext* context, const ::grpc::ByteBuffer* request, ::mavsdk::rpc::core::ListRunningPluginsResponse* response, std::function<void(::grpc::Status)>) override;
      void ListRunningPlugins(::grpc::ClientContext* context, const ::mavsdk::rpc::core::ListRunningPluginsRequest* request, ::mavsdk::rpc::core::ListRunningPluginsResponse* response, ::grpc::experimental::ClientUnaryReactor* reactor) override;
      void ListRunningPlugins(::grpc::ClientContext* context, const ::grpc::ByteBuffer* request, ::mavsdk::rpc::core::ListRunningPluginsResponse* response, ::grpc::experimental::ClientUnaryReactor* reactor) override;
     private:
      friend class Stub;
      explicit experimental_async(Stub* stub): stub_(stub) { }
      Stub* stub() { return stub_; }
      Stub* stub_;
    };
    class experimental_async_interface* experimental_async() override { return &async_stub_; }

   private:
    std::shared_ptr< ::grpc::ChannelInterface> channel_;
    class experimental_async async_stub_{this};
    ::grpc::ClientReader< ::mavsdk::rpc::core::ConnectionStateResponse>* SubscribeConnectionStateRaw(::grpc::ClientContext* context, const ::mavsdk::rpc::core::SubscribeConnectionStateRequest& request) override;
    ::grpc::ClientAsyncReader< ::mavsdk::rpc::core::ConnectionStateResponse>* AsyncSubscribeConnectionStateRaw(::grpc::ClientContext* context, const ::mavsdk::rpc::core::SubscribeConnectionStateRequest& request, ::grpc::CompletionQueue* cq, void* tag) override;
    ::grpc::ClientAsyncReader< ::mavsdk::rpc::core::ConnectionStateResponse>* PrepareAsyncSubscribeConnectionStateRaw(::grpc::ClientContext* context, const ::mavsdk::rpc::core::SubscribeConnectionStateRequest& request, ::grpc::CompletionQueue* cq) override;
    ::grpc::ClientAsyncResponseReader< ::mavsdk::rpc::core::ListRunningPluginsResponse>* AsyncListRunningPluginsRaw(::grpc::ClientContext* context, const ::mavsdk::rpc::core::ListRunningPluginsRequest& request, ::grpc::CompletionQueue* cq) override;
    ::grpc::ClientAsyncResponseReader< ::mavsdk::rpc::core::ListRunningPluginsResponse>* PrepareAsyncListRunningPluginsRaw(::grpc::ClientContext* context, const ::mavsdk::rpc::core::ListRunningPluginsRequest& request, ::grpc::CompletionQueue* cq) override;
    const ::grpc::internal::RpcMethod rpcmethod_SubscribeConnectionState_;
    const ::grpc::internal::RpcMethod rpcmethod_ListRunningPlugins_;
  };
  static std::unique_ptr<Stub> NewStub(const std::shared_ptr< ::grpc::ChannelInterface>& channel, const ::grpc::StubOptions& options = ::grpc::StubOptions());

  class Service : public ::grpc::Service {
   public:
    Service();
    virtual ~Service();
    // Subscribe to 'connection state' updates.
    virtual ::grpc::Status SubscribeConnectionState(::grpc::ServerContext* context, const ::mavsdk::rpc::core::SubscribeConnectionStateRequest* request, ::grpc::ServerWriter< ::mavsdk::rpc::core::ConnectionStateResponse>* writer);
    // Get a list of currently running plugins.
    virtual ::grpc::Status ListRunningPlugins(::grpc::ServerContext* context, const ::mavsdk::rpc::core::ListRunningPluginsRequest* request, ::mavsdk::rpc::core::ListRunningPluginsResponse* response);
  };
  template <class BaseClass>
  class WithAsyncMethod_SubscribeConnectionState : public BaseClass {
   private:
    void BaseClassMustBeDerivedFromService(const Service* /*service*/) {}
   public:
    WithAsyncMethod_SubscribeConnectionState() {
      ::grpc::Service::MarkMethodAsync(0);
    }
    ~WithAsyncMethod_SubscribeConnectionState() override {
      BaseClassMustBeDerivedFromService(this);
    }
    // disable synchronous version of this method
    ::grpc::Status SubscribeConnectionState(::grpc::ServerContext* /*context*/, const ::mavsdk::rpc::core::SubscribeConnectionStateRequest* /*request*/, ::grpc::ServerWriter< ::mavsdk::rpc::core::ConnectionStateResponse>* /*writer*/) override {
      abort();
      return ::grpc::Status(::grpc::StatusCode::UNIMPLEMENTED, "");
    }
    void RequestSubscribeConnectionState(::grpc::ServerContext* context, ::mavsdk::rpc::core::SubscribeConnectionStateRequest* request, ::grpc::ServerAsyncWriter< ::mavsdk::rpc::core::ConnectionStateResponse>* writer, ::grpc::CompletionQueue* new_call_cq, ::grpc::ServerCompletionQueue* notification_cq, void *tag) {
      ::grpc::Service::RequestAsyncServerStreaming(0, context, request, writer, new_call_cq, notification_cq, tag);
    }
  };
  template <class BaseClass>
  class WithAsyncMethod_ListRunningPlugins : public BaseClass {
   private:
    void BaseClassMustBeDerivedFromService(const Service* /*service*/) {}
   public:
    WithAsyncMethod_ListRunningPlugins() {
      ::grpc::Service::MarkMethodAsync(1);
    }
    ~WithAsyncMethod_ListRunningPlugins() override {
      BaseClassMustBeDerivedFromService(this);
    }
    // disable synchronous version of this method
    ::grpc::Status ListRunningPlugins(::grpc::ServerContext* /*context*/, const ::mavsdk::rpc::core::ListRunningPluginsRequest* /*request*/, ::mavsdk::rpc::core::ListRunningPluginsResponse* /*response*/) override {
      abort();
      return ::grpc::Status(::grpc::StatusCode::UNIMPLEMENTED, "");
    }
    void RequestListRunningPlugins(::grpc::ServerContext* context, ::mavsdk::rpc::core::ListRunningPluginsRequest* request, ::grpc::ServerAsyncResponseWriter< ::mavsdk::rpc::core::ListRunningPluginsResponse>* response, ::grpc::CompletionQueue* new_call_cq, ::grpc::ServerCompletionQueue* notification_cq, void *tag) {
      ::grpc::Service::RequestAsyncUnary(1, context, request, response, new_call_cq, notification_cq, tag);
    }
  };
  typedef WithAsyncMethod_SubscribeConnectionState<WithAsyncMethod_ListRunningPlugins<Service > > AsyncService;
  template <class BaseClass>
  class ExperimentalWithCallbackMethod_SubscribeConnectionState : public BaseClass {
   private:
    void BaseClassMustBeDerivedFromService(const Service* /*service*/) {}
   public:
    ExperimentalWithCallbackMethod_SubscribeConnectionState() {
      ::grpc::Service::experimental().MarkMethodCallback(0,
        new ::grpc_impl::internal::CallbackServerStreamingHandler< ::mavsdk::rpc::core::SubscribeConnectionStateRequest, ::mavsdk::rpc::core::ConnectionStateResponse>(
          [this](::grpc::experimental::CallbackServerContext* context, const ::mavsdk::rpc::core::SubscribeConnectionStateRequest* request) { return this->SubscribeConnectionState(context, request); }));
    }
    ~ExperimentalWithCallbackMethod_SubscribeConnectionState() override {
      BaseClassMustBeDerivedFromService(this);
    }
    // disable synchronous version of this method
    ::grpc::Status SubscribeConnectionState(::grpc::ServerContext* /*context*/, const ::mavsdk::rpc::core::SubscribeConnectionStateRequest* /*request*/, ::grpc::ServerWriter< ::mavsdk::rpc::core::ConnectionStateResponse>* /*writer*/) override {
      abort();
      return ::grpc::Status(::grpc::StatusCode::UNIMPLEMENTED, "");
    }
    virtual ::grpc::experimental::ServerWriteReactor< ::mavsdk::rpc::core::ConnectionStateResponse>* SubscribeConnectionState(::grpc::experimental::CallbackServerContext* /*context*/, const ::mavsdk::rpc::core::SubscribeConnectionStateRequest* /*request*/) { return nullptr; }
  };
  template <class BaseClass>
  class ExperimentalWithCallbackMethod_ListRunningPlugins : public BaseClass {
   private:
    void BaseClassMustBeDerivedFromService(const Service* /*service*/) {}
   public:
    ExperimentalWithCallbackMethod_ListRunningPlugins() {
      ::grpc::Service::experimental().MarkMethodCallback(1,
        new ::grpc_impl::internal::CallbackUnaryHandler< ::mavsdk::rpc::core::ListRunningPluginsRequest, ::mavsdk::rpc::core::ListRunningPluginsResponse>(
          [this](::grpc::experimental::CallbackServerContext* context, const ::mavsdk::rpc::core::ListRunningPluginsRequest* request, ::mavsdk::rpc::core::ListRunningPluginsResponse* response) { return this->ListRunningPlugins(context, request, response); }));}
    void SetMessageAllocatorFor_ListRunningPlugins(
        ::grpc::experimental::MessageAllocator< ::mavsdk::rpc::core::ListRunningPluginsRequest, ::mavsdk::rpc::core::ListRunningPluginsResponse>* allocator) {
      static_cast<::grpc_impl::internal::CallbackUnaryHandler< ::mavsdk::rpc::core::ListRunningPluginsRequest, ::mavsdk::rpc::core::ListRunningPluginsResponse>*>(
          ::grpc::Service::experimental().GetHandler(1))
              ->SetMessageAllocator(allocator);
    }
    ~ExperimentalWithCallbackMethod_ListRunningPlugins() override {
      BaseClassMustBeDerivedFromService(this);
    }
    // disable synchronous version of this method
    ::grpc::Status ListRunningPlugins(::grpc::ServerContext* /*context*/, const ::mavsdk::rpc::core::ListRunningPluginsRequest* /*request*/, ::mavsdk::rpc::core::ListRunningPluginsResponse* /*response*/) override {
      abort();
      return ::grpc::Status(::grpc::StatusCode::UNIMPLEMENTED, "");
    }
    virtual ::grpc::experimental::ServerUnaryReactor* ListRunningPlugins(::grpc::experimental::CallbackServerContext* /*context*/, const ::mavsdk::rpc::core::ListRunningPluginsRequest* /*request*/, ::mavsdk::rpc::core::ListRunningPluginsResponse* /*response*/) { return nullptr; }
  };
  typedef ExperimentalWithCallbackMethod_SubscribeConnectionState<ExperimentalWithCallbackMethod_ListRunningPlugins<Service > > ExperimentalCallbackService;
  template <class BaseClass>
  class WithGenericMethod_SubscribeConnectionState : public BaseClass {
   private:
    void BaseClassMustBeDerivedFromService(const Service* /*service*/) {}
   public:
    WithGenericMethod_SubscribeConnectionState() {
      ::grpc::Service::MarkMethodGeneric(0);
    }
    ~WithGenericMethod_SubscribeConnectionState() override {
      BaseClassMustBeDerivedFromService(this);
    }
    // disable synchronous version of this method
    ::grpc::Status SubscribeConnectionState(::grpc::ServerContext* /*context*/, const ::mavsdk::rpc::core::SubscribeConnectionStateRequest* /*request*/, ::grpc::ServerWriter< ::mavsdk::rpc::core::ConnectionStateResponse>* /*writer*/) override {
      abort();
      return ::grpc::Status(::grpc::StatusCode::UNIMPLEMENTED, "");
    }
  };
  template <class BaseClass>
  class WithGenericMethod_ListRunningPlugins : public BaseClass {
   private:
    void BaseClassMustBeDerivedFromService(const Service* /*service*/) {}
   public:
    WithGenericMethod_ListRunningPlugins() {
      ::grpc::Service::MarkMethodGeneric(1);
    }
    ~WithGenericMethod_ListRunningPlugins() override {
      BaseClassMustBeDerivedFromService(this);
    }
    // disable synchronous version of this method
    ::grpc::Status ListRunningPlugins(::grpc::ServerContext* /*context*/, const ::mavsdk::rpc::core::ListRunningPluginsRequest* /*request*/, ::mavsdk::rpc::core::ListRunningPluginsResponse* /*response*/) override {
      abort();
      return ::grpc::Status(::grpc::StatusCode::UNIMPLEMENTED, "");
    }
  };
  template <class BaseClass>
  class WithRawMethod_SubscribeConnectionState : public BaseClass {
   private:
    void BaseClassMustBeDerivedFromService(const Service* /*service*/) {}
   public:
    WithRawMethod_SubscribeConnectionState() {
      ::grpc::Service::MarkMethodRaw(0);
    }
    ~WithRawMethod_SubscribeConnectionState() override {
      BaseClassMustBeDerivedFromService(this);
    }
    // disable synchronous version of this method
    ::grpc::Status SubscribeConnectionState(::grpc::ServerContext* /*context*/, const ::mavsdk::rpc::core::SubscribeConnectionStateRequest* /*request*/, ::grpc::ServerWriter< ::mavsdk::rpc::core::ConnectionStateResponse>* /*writer*/) override {
      abort();
      return ::grpc::Status(::grpc::StatusCode::UNIMPLEMENTED, "");
    }
    void RequestSubscribeConnectionState(::grpc::ServerContext* context, ::grpc::ByteBuffer* request, ::grpc::ServerAsyncWriter< ::grpc::ByteBuffer>* writer, ::grpc::CompletionQueue* new_call_cq, ::grpc::ServerCompletionQueue* notification_cq, void *tag) {
      ::grpc::Service::RequestAsyncServerStreaming(0, context, request, writer, new_call_cq, notification_cq, tag);
    }
  };
  template <class BaseClass>
  class WithRawMethod_ListRunningPlugins : public BaseClass {
   private:
    void BaseClassMustBeDerivedFromService(const Service* /*service*/) {}
   public:
    WithRawMethod_ListRunningPlugins() {
      ::grpc::Service::MarkMethodRaw(1);
    }
    ~WithRawMethod_ListRunningPlugins() override {
      BaseClassMustBeDerivedFromService(this);
    }
    // disable synchronous version of this method
    ::grpc::Status ListRunningPlugins(::grpc::ServerContext* /*context*/, const ::mavsdk::rpc::core::ListRunningPluginsRequest* /*request*/, ::mavsdk::rpc::core::ListRunningPluginsResponse* /*response*/) override {
      abort();
      return ::grpc::Status(::grpc::StatusCode::UNIMPLEMENTED, "");
    }
    void RequestListRunningPlugins(::grpc::ServerContext* context, ::grpc::ByteBuffer* request, ::grpc::ServerAsyncResponseWriter< ::grpc::ByteBuffer>* response, ::grpc::CompletionQueue* new_call_cq, ::grpc::ServerCompletionQueue* notification_cq, void *tag) {
      ::grpc::Service::RequestAsyncUnary(1, context, request, response, new_call_cq, notification_cq, tag);
    }
  };
  template <class BaseClass>
  class ExperimentalWithRawCallbackMethod_SubscribeConnectionState : public BaseClass {
   private:
    void BaseClassMustBeDerivedFromService(const Service* /*service*/) {}
   public:
    ExperimentalWithRawCallbackMethod_SubscribeConnectionState() {
      ::grpc::Service::experimental().MarkMethodRawCallback(0,
        new ::grpc_impl::internal::CallbackServerStreamingHandler< ::grpc::ByteBuffer, ::grpc::ByteBuffer>(
          [this](::grpc::experimental::CallbackServerContext* context, const::grpc::ByteBuffer* request) { return this->SubscribeConnectionState(context, request); }));
    }
    ~ExperimentalWithRawCallbackMethod_SubscribeConnectionState() override {
      BaseClassMustBeDerivedFromService(this);
    }
    // disable synchronous version of this method
    ::grpc::Status SubscribeConnectionState(::grpc::ServerContext* /*context*/, const ::mavsdk::rpc::core::SubscribeConnectionStateRequest* /*request*/, ::grpc::ServerWriter< ::mavsdk::rpc::core::ConnectionStateResponse>* /*writer*/) override {
      abort();
      return ::grpc::Status(::grpc::StatusCode::UNIMPLEMENTED, "");
    }
    virtual ::grpc::experimental::ServerWriteReactor< ::grpc::ByteBuffer>* SubscribeConnectionState(::grpc::experimental::CallbackServerContext* /*context*/, const ::grpc::ByteBuffer* /*request*/) { return nullptr; }
  };
  template <class BaseClass>
  class ExperimentalWithRawCallbackMethod_ListRunningPlugins : public BaseClass {
   private:
    void BaseClassMustBeDerivedFromService(const Service* /*service*/) {}
   public:
    ExperimentalWithRawCallbackMethod_ListRunningPlugins() {
      ::grpc::Service::experimental().MarkMethodRawCallback(1,
        new ::grpc_impl::internal::CallbackUnaryHandler< ::grpc::ByteBuffer, ::grpc::ByteBuffer>(
          [this](::grpc::experimental::CallbackServerContext* context, const ::grpc::ByteBuffer* request, ::grpc::ByteBuffer* response) { return this->ListRunningPlugins(context, request, response); }));
    }
    ~ExperimentalWithRawCallbackMethod_ListRunningPlugins() override {
      BaseClassMustBeDerivedFromService(this);
    }
    // disable synchronous version of this method
    ::grpc::Status ListRunningPlugins(::grpc::ServerContext* /*context*/, const ::mavsdk::rpc::core::ListRunningPluginsRequest* /*request*/, ::mavsdk::rpc::core::ListRunningPluginsResponse* /*response*/) override {
      abort();
      return ::grpc::Status(::grpc::StatusCode::UNIMPLEMENTED, "");
    }
    virtual ::grpc::experimental::ServerUnaryReactor* ListRunningPlugins(::grpc::experimental::CallbackServerContext* /*context*/, const ::grpc::ByteBuffer* /*request*/, ::grpc::ByteBuffer* /*response*/) { return nullptr; }
  };
  template <class BaseClass>
  class WithStreamedUnaryMethod_ListRunningPlugins : public BaseClass {
   private:
    void BaseClassMustBeDerivedFromService(const Service* /*service*/) {}
   public:
    WithStreamedUnaryMethod_ListRunningPlugins() {
      ::grpc::Service::MarkMethodStreamed(1,
        new ::grpc::internal::StreamedUnaryHandler< ::mavsdk::rpc::core::ListRunningPluginsRequest, ::mavsdk::rpc::core::ListRunningPluginsResponse>(std::bind(&WithStreamedUnaryMethod_ListRunningPlugins<BaseClass>::StreamedListRunningPlugins, this, std::placeholders::_1, std::placeholders::_2)));
    }
    ~WithStreamedUnaryMethod_ListRunningPlugins() override {
      BaseClassMustBeDerivedFromService(this);
    }
    // disable regular version of this method
    ::grpc::Status ListRunningPlugins(::grpc::ServerContext* /*context*/, const ::mavsdk::rpc::core::ListRunningPluginsRequest* /*request*/, ::mavsdk::rpc::core::ListRunningPluginsResponse* /*response*/) override {
      abort();
      return ::grpc::Status(::grpc::StatusCode::UNIMPLEMENTED, "");
    }
    // replace default version of method with streamed unary
    virtual ::grpc::Status StreamedListRunningPlugins(::grpc::ServerContext* context, ::grpc::ServerUnaryStreamer< ::mavsdk::rpc::core::ListRunningPluginsRequest,::mavsdk::rpc::core::ListRunningPluginsResponse>* server_unary_streamer) = 0;
  };
  typedef WithStreamedUnaryMethod_ListRunningPlugins<Service > StreamedUnaryService;
  template <class BaseClass>
  class WithSplitStreamingMethod_SubscribeConnectionState : public BaseClass {
   private:
    void BaseClassMustBeDerivedFromService(const Service* /*service*/) {}
   public:
    WithSplitStreamingMethod_SubscribeConnectionState() {
      ::grpc::Service::MarkMethodStreamed(0,
        new ::grpc::internal::SplitServerStreamingHandler< ::mavsdk::rpc::core::SubscribeConnectionStateRequest, ::mavsdk::rpc::core::ConnectionStateResponse>(std::bind(&WithSplitStreamingMethod_SubscribeConnectionState<BaseClass>::StreamedSubscribeConnectionState, this, std::placeholders::_1, std::placeholders::_2)));
    }
    ~WithSplitStreamingMethod_SubscribeConnectionState() override {
      BaseClassMustBeDerivedFromService(this);
    }
    // disable regular version of this method
    ::grpc::Status SubscribeConnectionState(::grpc::ServerContext* /*context*/, const ::mavsdk::rpc::core::SubscribeConnectionStateRequest* /*request*/, ::grpc::ServerWriter< ::mavsdk::rpc::core::ConnectionStateResponse>* /*writer*/) override {
      abort();
      return ::grpc::Status(::grpc::StatusCode::UNIMPLEMENTED, "");
    }
    // replace default version of method with split streamed
    virtual ::grpc::Status StreamedSubscribeConnectionState(::grpc::ServerContext* context, ::grpc::ServerSplitStreamer< ::mavsdk::rpc::core::SubscribeConnectionStateRequest,::mavsdk::rpc::core::ConnectionStateResponse>* server_split_streamer) = 0;
  };
  typedef WithSplitStreamingMethod_SubscribeConnectionState<Service > SplitStreamedService;
  typedef WithSplitStreamingMethod_SubscribeConnectionState<WithStreamedUnaryMethod_ListRunningPlugins<Service > > StreamedService;
};

}  // namespace core
}  // namespace rpc
}  // namespace mavsdk


#endif  // GRPC_core_2fcore_2eproto__INCLUDED
