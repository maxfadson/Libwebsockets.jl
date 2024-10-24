function rewrite!(e::Expr)
    if e.head == "function"
        if e.args[1] == "lws_xos(xos)"
            println("Renaming lws_dll2_owner to lws_dll2_owner_fn")
            e.args[1] = "lws_xos_fn(xos)"
        elseif e.args[1] == :lws_threadpool_task_status
            println("Renaming lws_threadpool_task_status to lws_threadpool_task_status_fn")
            e.args[1] = :lws_threadpool_task_status_fn
        elseif e.args[1] == :lws_tokenize
            println("Renaming lws_tokenize to lws_tokenize_fn")
            e.args[1] = :lws_tokenize_fn
        elseif e.args[1] == :lws_xos
            println("Renaming lws_xos to lws_xos_fn")
            e.args[1] = :lws_xos_fn
        end
    end
end

function rewrite!(dag::ExprDAG)
    replace!(get_nodes(dag)) do node
        if node.id in # [exported symbol of xxx_jll]
            return ExprNode(node.id, Generators.Skip(), node.cursor, Expr[], node.adj)
        end
        return node
    end
end

# Apply rewrite to the DAG
function rewrite!(dag::ExprDAG)
    for node in get_nodes(dag)
        for expr in get_exprs(node)
            rewrite!(expr)
        end
    end
end