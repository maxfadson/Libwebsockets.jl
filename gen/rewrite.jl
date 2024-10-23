function rewrite!(expr::Expr)
    if expr.head == :function && expr.args[1] == :lws_dll2_owner
        expr.args[1] = :lws_dll2_owner_fn
    end

    if expr.head == :function && expr.args[1] == :lws_threadpool_task_status
        expr.args[1] = :lws_threadpool_task_status_fn
    end

    if expr.head == :function && expr.args[1] == :lws_tokenize
        expr.args[1] = :lws_tokenize_fn
    end

    if expr.head == :function && expr.args[1] == :lws_xos
        expr.args[1] = :lws_xos_fn
    end
end

function rewrite!(dag::ExprDAG)
    for node in get_nodes(dag)
        for expr in get_exprs(node)
            rewrite!(expr)
        end
    end
end