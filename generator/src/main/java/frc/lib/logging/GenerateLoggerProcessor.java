package frc.lib.logging;

import java.io.IOException;
import java.io.UncheckedIOException;
import java.lang.reflect.Method;
import java.lang.reflect.Parameter;
import java.lang.reflect.Type;
import java.lang.reflect.TypeVariable;
import java.util.List;
import java.util.Optional;
import java.util.Set;

import javax.annotation.processing.AbstractProcessor;
import javax.annotation.processing.RoundEnvironment;
import javax.lang.model.SourceVersion;
import javax.lang.model.element.Element;
import javax.lang.model.element.ExecutableElement;
import javax.lang.model.element.Modifier;
import javax.lang.model.element.TypeElement;
import javax.lang.model.element.VariableElement;
import javax.lang.model.util.ElementFilter;

import org.littletonrobotics.junction.Logger;

import com.palantir.javapoet.ClassName;
import com.palantir.javapoet.JavaFile;
import com.palantir.javapoet.MethodSpec;
import com.palantir.javapoet.TypeName;
import com.palantir.javapoet.TypeSpec;
import com.palantir.javapoet.TypeVariableName;

public class GenerateLoggerProcessor extends AbstractProcessor {

    @Override
    public boolean process(Set<? extends TypeElement> annotations, RoundEnvironment roundEnv) {
        Optional<? extends TypeElement> annotationOptional = annotations.stream()
                .filter(te -> "GenerateLogger".equals(te.getSimpleName().toString()))
                .findFirst();
        if (!annotationOptional.isPresent()) {
            return false;
        }

        TypeElement annotation = annotationOptional.get();
        for (Element e : roundEnv.getElementsAnnotatedWith(annotation)) {
            try {
                buildClass((TypeElement) e);
            } catch (IOException e1) {
                throw new UncheckedIOException(e1);
            }
        }
        return true;
    }

    private void buildClass(TypeElement target) throws IOException {
        TypeSpec.Builder classBuilder = TypeSpec.classBuilder("CustomLogger")
                .addModifiers(Modifier.PUBLIC, Modifier.FINAL)
                .superclass(target.asType());

        for (ExecutableElement targetConstructor : ElementFilter.constructorsIn(target.getEnclosedElements())) {
            MethodSpec.Builder constructor = MethodSpec.constructorBuilder();
            constructor.addCode("super(");
            List<? extends VariableElement> parameters = targetConstructor.getParameters();
            for (int i = 0; i < parameters.size(); i++) {
                constructor.addParameter(TypeName.get(parameters.get(i).asType()),
                        parameters.get(i).getSimpleName().toString());
                constructor.addCode("$L", parameters.get(i).getSimpleName());
                if (i < parameters.size() - 1) {
                    constructor.addCode(", ");
                }
            }
            constructor.addCode(");");
            classBuilder.addMethod(constructor.build());
        }

        for (Method loggerMethod : Logger.class.getDeclaredMethods()) {
            for (LogMode mode : LogMode.values()) {
                if ("recordOutput".equals(loggerMethod.getName())) {
                    MethodSpec.Builder methodBuilder = MethodSpec.methodBuilder(mode.name().toLowerCase())
                            .addModifiers(Modifier.PUBLIC)
                            .returns(ClassName.get("frc.lib.logging", "CustomLogger"));

                    for (TypeVariable<Method> i : loggerMethod.getTypeParameters()) {
                        methodBuilder.addTypeVariable(TypeVariableName.get(i).withBounds(i.getBounds()));
                    }

                    Type[] parameterTypes = loggerMethod.getGenericParameterTypes();
                    Parameter[] parameters = loggerMethod.getParameters();
                    for (int i = 0; i < parameters.length; i++) {
                        methodBuilder.addParameter(parameterTypes[i], parameters[i].getName());
                    }

                    methodBuilder.beginControlFlow("if ($L())", mode.methodName);
                    methodBuilder.addCode("$T.recordOutput(getOutputLogPath($L)", Logger.class,
                            parameters[0].getName());
                    for (int i = 1; i < parameters.length; i++) {
                        methodBuilder.addCode(", $L", parameters[i].getName());
                    }
                    methodBuilder.addCode(");\n");
                    methodBuilder.endControlFlow();

                    methodBuilder.addStatement("return this");

                    classBuilder.addMethod(methodBuilder.build());
                }
            }
        }

        JavaFile file = JavaFile.builder("frc.lib.logging", classBuilder.build()).build();
        file.writeTo(processingEnv.getFiler());
    }

    @Override
    public SourceVersion getSupportedSourceVersion() {
        return SourceVersion.latestSupported();
    }

    @Override
    public Set<String> getSupportedAnnotationTypes() {
        return Set.of("frc.lib.logging.GenerateLogger");
    }

    enum LogMode {
        DEBUG("shouldLogDebug"),
        DASHBOARD("shouldLogDashboard");

        private final String methodName;

        LogMode(String methodName) {
            this.methodName = methodName;
        }
    }
}
